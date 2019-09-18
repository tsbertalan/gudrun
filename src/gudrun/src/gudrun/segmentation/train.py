#!/usr/bin/env python
# coding: utf-8

# # U-Net trained from scratch
# 
# This notebook implements the U-Net semantic segmentation architecture from [Ronneberger, Fischer, and Brox](https://arxiv.org/abs/1505.04597). The default size of the network has been reduced for training on a smaller dataset.
# 
# However, in Ronneberger et al., other training details are added, including a pixelwise weighting factor to give some areas more importance in the loss function than others, and a data augmentation procedure to increase the effective size of the dataset. These are not employed here (though an augmentation scheme, in particular, might be a helpful addition).
# 
# In addition, it would be helpful to track the IOU quality metric during training, and to carve out a validation set to detect overfitting, since the network is somewhat large compared to the number-of-images in our JHU dataset (though not so large compared to the raw number of pixels).

# ## Do imports and write a little helper code.

## Do standard imports.
import numpy as np, matplotlib.pyplot as plt
import tensorflow as tf
from tqdm import tqdm as tqdm  # progressbar library
from PIL import Image  # Python Imaging Library
from os import system

describe = lambda name, arr: print('%s:' % name, arr.shape, arr.dtype, [arr.min(), arr.max()])

## Load data (change path to suit).
# datafile_path = '/home/tsbertalan/data/jhu_semantic_segmentation/jhu_data/jhu_data-439.npz'
datafile_path = '/home/tsbertalan/Dropbox/data/gudrun/tp_apartment_labels.npz'
data_npz = np.load(datafile_path, allow_pickle=True)
if 'X' in data_npz:
    X = data_npz['X']
    Y = data_npz['Y']
else:
    assert 'images' in data_npz, list(data_npz.keys())
    X = data_npz['images']
    Y = data_npz['label_images']
    X = np.array([x for (x,y) in zip(X, Y) if len(y) > 0])
    from functools import reduce
    Y = np.array([reduce(np.logical_and, [a.astype('bool') for a in y]) for y in Y if len(y) > 0])

if len(Y.shape) == 3:
    Y = np.stack([
        Y, np.logical_not(Y)
    ], axis=-1)

X = X.astype('float32')
Y = Y.astype('bool')

describe('X', X)
describe('Y', Y)

## Define a couple plotting functions.

#  Some default colors.
_colors = np.array([
    [0.88798955, 0.17623193, 0.42475507],
    [0.37974051, 0.69121509, 0.61483237],
    [0.02141717, 0.48236297, 0.0180039 ],
    [0.31523748, 0.05414061, 0.36440007],
    [0.81117673, 0.26315918, 0.08352317],
    [0.51117777, 0.59272035, 0.75051711],
    [0.06888136, 0.04488548, 0.47847962],
    [0.71159935, 0.3167025 , 0.30139559]
])


def colored_onehot(onehot, colors):
    onehot = np.array(onehot, dtype=float)
    onehot -= onehot.min()
    onehot /= onehot.max()

    colored = np.zeros((*onehot.shape[:2], 3))
    for i in range(onehot.shape[-1]):
        mask = onehot[..., i]

        mask = np.stack([mask]*3, axis=2)
        color = colors[i]
        color /= np.linalg.norm(color)
        for i in range(3):
            mask[..., i] *= color[i]
        colored += mask

    colored[colored>1] = 1

    colored *= 255.

    return Image.fromarray(colored.astype('uint8'))


def overlay_prediction(predictor, probabilities, thresh=.6, image_fraction=.4):
    predictor = np.array(predictor)
    if predictor.dtype == 'uint8':
        predictor = predictor.astype(float) / 255.

    prediction = probabilities > thresh

    colors = np.array(colored_onehot(prediction, _colors)).astype(float) / 255.

    mixed = image_fraction * predictor + (1.-image_fraction) * colors

    return Image.fromarray((mixed*255.).astype('uint8'))


## Some NN functions
# for tracking loss history and displaying a progress bar during training
class ProgresbarCallback(tf.keras.callbacks.Callback):
    
    def __init__(self, nepochs, historian):
        self._bar = tqdm(total=nepochs, unit='epochs')
        self.historian = historian
        
    def on_epoch_end(self, epoch, logs={}):
        self.historian.record(logs['loss'])
        self._bar.update()
     

class Historian:
    
    def __init__(self):
        self._loss_history = []
    
    def record(self, loss):
        self._loss_history.append(loss)
        
    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(self._loss_history)
        ax.set_xlabel('epoch')
        ax.set_ylabel('loss')
        ax.set_yscale('log');
        return fig, ax


# Instantiate our loss-tracker here (that is, re-instantiate it if this code is changed).
historian = Historian()


# We'll import some things directly from Keras since we 
def crop(source, template):
    """Given a larger image tensor and a smaller one, crop the larger.
    
    Both tensors should be in (batch?, height, width, channels) shape.
    We try to crop symmetrically in both rows and columns, but, if the
    amount of slack height or width is odd, we crop one more row or column on one side.
    """
    B, H, W, C = source.shape
    b, h, w, c = template.shape
    
    # How much we hope the crop should be on each side.
    dy = .5 * int(H - h)
    dx = .5 * int(W - w)
    def ab(d):
        """With a float delta, return the two ints that add up to 2*d."""
        a = np.math.floor(d)
        b = int(d*2 - a)
        return a, b
    left, right = ab(dx)
    bottom, top = ab(dy)
    
    # Crop.
    slicer = tf.keras.layers.Lambda(lambda x: x[:, bottom:-top, left:-right, :])
    return slicer(source)
  

def intshape(tensor):
    """For printing mostly; return a tensor's shape as ints."""
    out = []
    for s in tensor.shape:
        try:
            s = int(s)
        except TypeError:
            s = -1
        out.append(s)
    return tuple(out)


class Block:
    """A group of several convolutional layers; repeated in several places in UNet."""
    
    def __init__(self, 
                 filter_counts=(8,8), filter_size=3, pool_size=2, activation='relu', 
                 upconv=False,
                 upconv_filter_counts=None,
                ):
        """
        filter_counts: How many filters to return from each of the convolutional layers
        filter_size: How wide the filters should be
        pool_size: How big the maxpool filters should be, 
            or, alternately (if upconv), how big the upscaling convolutional filters should be.
        activation: What activation function to use on the main convolutional layers
            (not used for the maxpooling or upscaling convolutions)
        upconv: Whether the auxiliary output should be maxpool or upconv of the main output
        upconv_filter_counts: If upconv, how many filters to use for the upconv.
        """
        self.filter_counts = filter_counts
        
        # Create the primary (activation-bearing) convolutional layers.
        self.convs = [
            tf.keras.layers.Conv2D(f, filter_size, activation=activation)
            for f in filter_counts
        ]
        
        # Create the layer for the auxiliary output.
        if not upconv:
            # Note that maxpool has no trainable weights.
            self.pool = tf.keras.layers.MaxPool2D(pool_size)
        else:
            # Note that, if upconv and we do this, we will also instantiate some extra weights.
            # So the "default" case (for e.g. the final block) should use maxpool,
            # since we won't use its auxiliary output, and so don't want to bother carrying around any extra weights.
            self.pool = tf.keras.layers.Conv2DTranspose(upconv_filter_counts, pool_size, strides=pool_size)

        print('Created a block with filter_counts=', filter_counts)
        
    def __call__(self, input_map, verbose=True):
        """Apply the block to a tensor (this should probably only be called once)."""
        self.input_map = input_map
        activations = [input_map]
        
        # Pass the input image/feature map through the main convolutional layers.
        for c in self.convs:
            activations.append(c(activations[-1]))
            
        # Create the primary output.
        self.hires = activations[-1]
        
        # Create the secondary output.
        self.lowres = self.pool(self.hires)
        
        # Save the activations for later introspection.
        self.activations = activations
        
        if verbose: print(intshape(input_map), '->', intshape(self.hires), 'and', intshape(self.lowres))
        return self.hires, self.lowres
        
        
def kerasConcat(tensors, axis=-1):
    layer = tf.keras.layers.Concatenate(axis=axis)
    return layer(tensors)


class Unet:
    
    # The original paper used these sizes,
    # which comes out to about 31 million trainable parameters!
    #block_sizes=[(64,64), (128,128), (256,256), (512,512)], pool_size=2,
    #bottom_block_size=(1024,1024),

    # We'll use these, to make our network smaller, so we can do larger batches
    # during training (and prediction), and train faster.
    # This uses a much more manageable 30 to 120 thousand parameters.
    def __init__(self, 
        block_sizes=[(8,8), (16,16), (32, 32)], pool_size=2,
        bottom_block_size=(64,64),
        n_classes=2,
        ):
        
        self.encoding_blocks = [
            Block(bs, pool_size=pool_size)
            for bs in block_sizes
        ]
        
        self.decoding_blocks = [
            # The "decoding" blocks include transposed convolution "upconv" layers
            # which will produce outputs of the same number of filters as the roughly
            # similarly-sized encoding blocks, which is why ywe need bs_next.
            Block(bs, pool_size=pool_size, upconv=True, upconv_filter_counts=bs_next[-1])
            for (bs, bs_next) in zip(
                block_sizes[::-1], # Loop over the encoding blocks in reverse order.
                block_sizes[::-1][1:]
            )
        ]
        
        #  We'll special-case the lowest block in the U shape, and the final output.
        self.bottom_block = Block(
            bottom_block_size, pool_size=pool_size, upconv=True,
            upconv_filter_counts=block_sizes[-1][-1],
        )
        
        # The final output block is a special case because (1) we don't want to create
        # any deconvolutional layer in it, and (2) we compute its number of filters differently
        # than the other decoding blocks.
        self.final_block = Block(
            block_sizes[0], pool_size=pool_size, # pool is unused here
        )
        
        # The "logits" in a classification model are the real numbers
        # ($\alpha \in [-\infty, \infty]$, in principle)
        # which are the precursors to the actual per-class probabilities,
        # which themselves must add up to one for each example
        # (or, for segmentation, for each pixel).
        self.logits_1x1 = tf.keras.layers.Conv2D(n_classes, 1)
        
    def __call__(self, input_image, verbose=True):
        """Actually construct the graph."""
        x = input_image
        
        if verbose: print('Encoder:')
        for block in self.encoding_blocks:
            _, x = block(x, verbose)
            
        if verbose: print('\nBottom block:')
        _, x = self.bottom_block(x, verbose)
                
        if verbose: print('\nDecoder:')
        for block, encblock in zip(self.decoding_blocks, self.encoding_blocks[::-1]):
            # Crop the carried-forward high-resolution feature maps from the encoding blocks
            # to be the same H,W as the upscaled decoding block feature maps.
            cropped = crop(encblock.hires, x)
            unconcatenated = x
            
            # Concatenate the cropped and the upscaled feature maps along their 3rd axis
            # (the channel axis in (batch?, height, width, channels)).
            x = kerasConcat([cropped, unconcatenated], axis=3)
            if verbose: print('concat', (intshape(cropped), intshape(unconcatenated)), '=', intshape(x))
            _, x = block(x, verbose)
            
        # Also do the final output block mostly like the decoding blocks.
        cropped = crop(self.encoding_blocks[0].hires, x)
        unconcatenated = x
        x = kerasConcat([cropped, unconcatenated], axis=3)
        if verbose: print('concat', (intshape(cropped), intshape(unconcatenated)), '=', intshape(x))
        final_feature_map, unused_maxpool = self.final_block(x, verbose)
        
        logits = self.logits_1x1(final_feature_map)
        if verbose: print('\nLogits shape:', intshape(logits))

        
        return tf.identity(logits, name='logits')


tf.keras.layers.Concatenate
n_classes = Y.shape[-1]
print('n_classes=', n_classes)


# Create an input tensor for our particular data's shape.
# Note--all the data I'm supplying is scaled to a particular shape,
# but, in principle, you could create a model with a flexible input shape
# and corresponding flexible target one-hot image shape,
# as long as (in TensorFlow 1.x) you batch same-shape images together.
# I understand that TensorFlow 2.x allows for ragged arrays, which might allow you to combine
# images of different shapes in one batch.
input_tensor = tf.keras.Input(shape=X[0].shape, name='input_tensor')

# The original paper used monochrome images of this shape. You can uncomment
# this and the shape parameters above to verify that the tensor shapes match
# the figure from the paper.
#input_tensor = tf.keras.Input(shape=(572,572,1))
unet = Unet(n_classes=n_classes)
logits = unet(input_tensor, verbose=True)

sm = tf.keras.layers.Softmax()

unresized_probability = tf.identity(sm(logits), name='unresized_probability')

unresized_probability_model = tf.keras.Model(
    inputs=input_tensor, 
    outputs=unresized_probability,
)

unresized_probability_model._make_predict_function()

# # Create an operation (with no learnable parameters) that rescales
# # the predicted class probabilities image to be the same size
# # as the ground-truth targets.
# resizer = tf.keras.layers.Lambda(
#     lambda x: tf.image.resize_images(
#         x, X[0].shape[:2], 
#         method=tf.image.ResizeMethod.NEAREST_NEIGHBOR
#     ))
# resized = resizer(logits)
# resized
resized = tf.image.resize_images(logits, X[0].shape[:2], method=tf.image.ResizeMethod.NEAREST_NEIGHBOR, name='resized')

# Create the per-class probabilities tensor. The "softmax" function takes a vector of real numbers, and returns a vector of numbers $\in[0,1]$ that all sum to one. By default, TensorFlow's `softmax` function does this along the last axis of a tensor.
# 
# Note that, if we get the predictions perfectly right, these "probability" vectors will be all 0s except for one 1 each--that is, "one-hot" encodings. This is how our ground-truth targets are encoded.
probabilities = tf.nn.softmax(resized, name='probabilities')

# For completeness, we can also define a tensor that returns the index of the largest probability at each pixel (though it's probably more useful to return the probabilities and let the user threshold these in NumPy code themselves).
predictions = tf.argmax(probabilities, axis=-1, name='predictions')

# Get our main model ready for training. Note that we use the `CategoricalCrossentropy` loss. Broady speaking, this is a loss function that measures the wrongness of a per-example categorial distribution. Though, mathematically, this can be computed with the probability tensor we defined above, for effective training, an alternative implementation that does some thresholding is needed.
# 
# I don't really like how, in Keras, you specify an "output" tensor and then just say "apply this loss to it". For classification, which output do you supply? `resized`? `probabilities`? `predictions`? I think the answer is normally `probabilities`; after which, under the hood, Keras walks up the tree to the corresponding logits, effectively removing the softmax "activation" function, and then does their crossentropy-with-logits there. However, to be explicit, I'm instead just using the full `tf.keras.losses.CategoricalCrossentropy` class with the `from_logits=True` argument, and passing the resized logits themselves.
# 
# We'll also use the Adam optimizer, since that's what people usually use (and It's always work best in all my applications).
model = tf.keras.Model(inputs=input_tensor, outputs=resized)
model.compile("adam", tf.keras.losses.CategoricalCrossentropy(from_logits=True))

# We will also make a `Model` object for the probabilities, but we don't need to `compile` this, [since we're not training it](github.com/keras-team/keras/issues/2621).
probability_model = tf.keras.Model(inputs=input_tensor, outputs=probabilities)


# We can now look at all the layers in our `model`, and see the tally of all trainable parameters.
# model.summary()

## Train.
# Note that, if you're using the full-size architecture from the paper, you'll probably need to pass a much smaller `batch_size` (like, 5) when calling `model.fit`, to keep within the size of your GPU memory.
nepochs = 1600

print('Training on X dataset of shape %s.' % (X.shape,))
cb = ProgresbarCallback(nepochs, historian)
chunksize = 25
nchunks = int(nepochs / chunksize)
assert chunksize == int(nepochs / nchunks), (chunksize, int(nepochs/nchunks))
interrupted = False
for chunk in range(nchunks):

    try:
        model.fit(X, Y, callbacks=[cb], epochs=chunksize, verbose=0, batch_size=32)

    except KeyboardInterrupt:
        print('User interupted.')
        interrupted = True

    print('Saving loss history plot.')
    f, a = historian.plot()
    f.savefig('loss_history.png')

    ## Examine the results.
    # If you scroll up and read the code for `overlay_prediction`, you'll see that we do this by thresholding the predicted class probabilities. So, at `thresh = 0.5`, everything gets a color, but, at higher thresholds, some regions of the image might get no assigned class.
    # i = 127
    i = 17

    predictions = probability_model.predict(X[i].reshape([1] + list(X.shape[1:])))

    thresh = .5

    describe('predictions', predictions)

    im = overlay_prediction(X[i], np.squeeze(predictions[0]), thresh=thresh)
    im.save('trained.png')

    ## Save the model.
    print('Checkpointing the model.')
    saver = tf.train.Saver()
    sess = tf.keras.backend.get_session()
    save_path = saver.save(sess, "Unet_model.ckpt")

    if interrupted: break

# converter = tf.lite.TFLiteConverter.from_session(sess, [input_tensor], [probabilities])

# # converter.inference_type = tf.lite.constants.QUANTIZED_UINT8
# # converter.quantized_input_stats = {input_tensor : (0., 1.)}  # mean, std_dev

# converter.optimizations = [tf.lite.Optimize.DEFAULT]
# # converter.target_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]

# def representative_dataset_gen():
#     for i in range(len(X)):
#         yield [X[i]]
# converter.representative_dataset = representative_dataset_gen

# tflite_model = converter.convert()

# with open('Unet_converted.tflite', 'wb') as f:
#     f.write(tflite_model)


# There doesn't appear to be a way to add a resize op to a Keras model and get it to save, so we'll just make a model that doesn't do that, and let the user do that themselves in scipy code after the fact.
if True:
    np.savez('X12.npz', X=X[-12:])

    #Our model takes BGR images.
    import cv2
    plt.imshow(cv2.cvtColor((X[0]*255).astype('uint8'), cv2.COLOR_BGR2RGB))

# system('ssh sigurd.vpn.tomsb.net "source ~/.virtualenvs/gudrun/bin/activate && cd ~/Dropbox/Projects/Online\\ Robocar\\ Class/handouts/7\\ FCN\\ Segmentation && DISPLAY=:1 python keras_load.py"')


# with open('Unet_arch.json', 'w') as f:
#     f.write(unresized_probability_model.to_json())
# unresized_probability_model.save_weights('./Unet_weights.h5')

keras_file = 'Unet.h5'

# This is convenient, but, due to an insistence on saving Python code in the file, is not portable across Python versions (for instance, if we want to use 2.7 in ROS):
unresized_probability_model.save(keras_file)

# tf.keras.models.save_model(unresized_probability_model, keras_file)
# converter = tf.lite.TFLiteConverter.from_keras_model_file(keras_file)
system('ls -lah')

