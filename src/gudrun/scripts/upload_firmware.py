#!/usr/bin/env python
from sys import argv

if argv[1] == 'encoder':
    from gudrun.encoder import Encoder
    e = Encoder(allow_default_ss=True, connect=False)
    e.flash()