#!/usr/bin/env python
from __future__ import print_function
import rospy, time, numpy as np

import matplotlib.pyplot as plt
plt.switch_backend('Qt4Agg')


from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class ReactiveOptimization(object):

    def __init__(self):

        rospy.init_node('reactive_optimization')
        print('ReactiveOptimization init')

        self.is_shutdown = False

        rospy.Subscriber('/camera/depth/points', PointCloud2, self.callback)

        while not rospy.is_shutdown():
            pc = self.point_cloud
            if pc is not None:
                f, a = self.top_down()
                # f.savefig('top_down.pdf')
                plt.pause(.05)
            else:
                rospy.rostime.wallsleep(0.5)

    @property
    def point_cloud(self):
        if not hasattr(self, '_point_cloud'):
            return None
        else:
            return self._point_cloud[:self.num_filled]

    def callback(self, msg):

        if not hasattr(self, '_point_cloud'):
            self._point_cloud = np.zeros((msg.height * msg.width, 3))

        pc = pc2.read_points(msg, skip_nans=False, field_names=('x', 'y', 'z'))
        for i, point in enumerate(pc):
            self.shape = (
                msg.height, 
                msg.width,
                3,
            )
            self._point_cloud[i, :] = point  # Includes NaNs
        self.num_filled = i + 1

        self.is_shutdown = True

    def flyaround(self, path='./point_cloud', dataskip=10, frameskip=None):
        from mpl_toolkits.mplot3d import Axes3D
        import tqdm, os

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        x, y, z = self.point_cloud[::dataskip].T
        print('Got', len(x), 'points to plot.')

        fig.colorbar(
            ax.scatter(x, z, y, c=z, s=1),
            ax=ax,
            label='$z$'
        )

        ax.set_xlabel('$x$')
        ax.set_ylabel('$z$')
        ax.set_zlabel('$y$')

        if frameskip is not None:

            def cmd(c):
                print('$ %s' % c)
                os.system(c)

            cmd('rm /tmp/pc_*.png')
            for k in tqdm.tqdm(range(0, 360, frameskip)):
                ax.view_init(azim=k)
                fig.savefig('/tmp/pc_%04d.png' % k)

            #cmd('ffmpeg -loglevel quiet -y -i /tmp/pc_%04d.png -crf 15' + ' "%s"' % path + '.mp4')
            cmd('convert -delay 20 /tmp/pc_*.png -loop 0 "%s"' % path + '.gif')
            cmd('convert -delay 20 /tmp/pc_*.png -loop 0 "%s"' % path + '.mp4')

        else:
            plt.show()

    def closest_index(self, angle, height_index=200):
        a = self.point_cloud.reshape(self.shape)
        X = a[height_index, :, 0]
        Y = a[height_index, :, 1]
        Z = a[height_index, :, 2]
        I = np.arange(len(X))

        allowed = np.logical_and(
            np.logical_not(np.isnan(X)),
            np.logical_not(np.isnan(Z)),
        )

        angles = np.arctan2(X[allowed], Z[allowed])
        index = np.argmin((angles - angle) ** 2)
        return I[allowed][index], X[allowed][index], Y[allowed][index], Z[allowed][index]

    def angle_to_index(self, angle, capped=True):
        #ang = m * ind + b
        m = 0.0016390736317718278
        b = -0.51482610557816832
        index = int((angle - b) / m)
        if capped:
            return max(min(index, self.shape[1]-1), 0)
        else:
            return index

    def get_slice(self, height_index_range=(190, 210)):
        a = self.point_cloud.reshape(self.shape)
        rows = a[height_index_range[0]:height_index_range[1], :, :]
        XYZ = np.empty((rows.shape[1], 3))
        for j in range(rows.shape[1]):
            col_x = rows[:, j, 0]
            col_y = rows[:, j, 0]
            col_z = rows[:, j, 2]
            allowed = np.logical_and(
                np.logical_not(np.isnan(col_x)),
                np.logical_not(np.isnan(col_z)),
            )
            XYZ[j, 0] = col_x[allowed].mean()
            XYZ[j, 1] = col_y[allowed].mean()
            XYZ[j, 2] = col_z[allowed].mean()
        return XYZ

    def distance_for_ray(self, angle, height_index=200):
        index_for_angle = self.angle_to_index(angle, capped=False)
        if index_for_angle < 0 or index_for_angle >= self.shape[1]:
            return np.nan

        # a = self.point_cloud.reshape(self.shape)
        # X = a[height_index, :, 0]
        # Z = a[height_index, :, 2]
        XYZ = self.get_slice((height_index-10, height_index+10))
        X = XYZ[:, 0]
        Z = XYZ[:, 2]

        # indices = np.logical_and(
        #     np.logical_not(np.isnan(X)),
        #     np.logical_not(np.isnan(Z)),
        # )
        # X = X[indices]
        # Z = Z[indices]

        D = np.sqrt(X ** 2 + Z ** 2)

        return D[index_for_angle]

        # Z_pred = X / np.tan(angle)
        # D_pred = np.sqrt(X ** 2 + Z_pred ** 2)
        # squared_deviations = (D - D_pred) ** 2

        # return D[np.argmin(squared_deviations)]

    def top_down(self, yradius=.025):
        y = self.point_cloud[:, 1]
        indices = np.logical_and(
            y < yradius,
            y > -yradius
        )
        x, y, z = self.point_cloud[indices].T

        print('Limited to', len(x), 'points.')


        # fig, ax = plt.subplots()
        # index = 200
        # a = self.point_cloud.reshape(self.shape)
        # xprofile = a[index, :, 0]
        # zprofile = a[index, :, 2]
        # distances = np.sqrt(xprofile ** 2 + zprofile ** 2)
        # ax.scatter(range(len(distances)), distances)
        # ax.set_ylabel('$d$')

        if not hasattr(self, 'fa'):
            fig, ax = plt.subplots()
            self.fa = fig, ax
        else:
            fig, ax = self.fa
        ax.cla()

        # sc = ax.scatter(x, z, 
        #     # c=y, 
        #     color='black',
        #     alpha=.25,
        #     s=1)
        # # fig.colorbar(
        # #     sc,
        # #     ax=ax,
        # #     label='$y$',
        # # )
        ax.set_xlabel('$x$')
        ax.set_ylabel('$z$')
        # ax.set_aspect('equal', 'datalim')

        XYZ = self.get_slice()

        ax.scatter(XYZ[:, 0], XYZ[:, 2], color='red', s=1)

        distances = []
        angles = np.linspace(-.35, .35, 100) * np.pi

        FREE_RAY_DISTANCE = 1.5

        endpoints = []

        for angle in angles:

            x0, z0 = 0, 0

            ray_length = self.distance_for_ray(angle)
            if np.isnan(ray_length).all():
                ray_length = FREE_RAY_DISTANCE
                color = 'green'
            else:
                color = 'black'

            distances.append(ray_length)

            z1 = ray_length * np.cos(angle)
            x1 = ray_length * np.sin(angle)

            # index, x1, y1, z1 = self.closest_index(angle)

            endpoints.append((x1, z1))

            # index = self.angle_to_index(angle, capped=False)
            # if index > 0 and index < self.shape[1]:
            #     a = self.point_cloud.reshape(self.shape)
            #     xp, yp, zp = a[200, index, :]
            #     # ax.scatter([xp], [zp], s=42, color='black')
            #     ax.plot([x1, xp], [z1, zp], color='red')

        distances = np.array(distances)
        scores = 1. / np.array(distances)

        search_radius = 5
        for i in range(len(scores)):
            if distances[i] >= FREE_RAY_DISTANCE:
                indices = [j for j in range(i-search_radius, i+search_radius+1) if j >= 0 and j < len(distances)]
                scores[i] = 1./distances[indices].mean()

        angle_factor = 3.0
        scores += np.abs(angles ** 4) / np.pi * angle_factor

        sizes = np.array(scores)
        sizes -= sizes.min(); sizes /= sizes.max()
        sizes *= -1; sizes += 1
        sizes *= 100
        sizes += 5

        endpoints = np.array(endpoints)
        sc = ax.scatter(endpoints[:, 0], endpoints[:, 1], s=sizes, c=scores)
        # fig.colorbar(sc, ax=ax, label='score')

        alphas = np.array(scores)
        min_alpha = .01
        alphas -= alphas.min(); alphas /= alphas.max(); 
        alphas *= -1; alphas += 1
        alphas *= (1 - min_alpha)
        alphas += min_alpha
        alphas *= .8

        i_best = np.argmin(scores)

        for i, (score, alpha, (x1, z1)) in enumerate(zip(scores, alphas, endpoints)):
            if score > scores.mean():
                color = 'green'
            else:
                color = 'black'
            x0 = z0 = 0
            best = i == i_best
            ax.plot(
                [x0, x1], [z0, z1], 
                color='magenta' if best else color,
                linestyle='-', alpha=alpha,
                linewidth=3 if best else 1
            )

        ax.set_xlim(-2, 2)
        ax.set_ylim(0, 2)


        # fig, ax = plt.subplots()
        # fig.colorbar(
        #     ax.scatter(x, y, c=z, s=1),
        #     ax=ax,
        #     label='$z$'
        # )
        # ax.set_xlabel('$x$')
        # ax.set_ylabel('$y$')

        # fig, ax = plt.subplots()
        # fig.colorbar(
        #     ax.scatter(x, y, c=np.arange(len(x)), s=1),
        #     ax=ax,
        #     label='index'
        # )
        # ax.set_xlabel('$x$')
        # ax.set_ylabel('$y$')

        # fig, axes = plt.subplots(ncols=3)
        # a = self.point_cloud.reshape(self.shape)

        # for i, ax in enumerate(axes):
        #     ax.imshow(a[:, :, i])

        return fig, ax



if __name__ == '__main__':

    ro = ReactiveOptimization()
