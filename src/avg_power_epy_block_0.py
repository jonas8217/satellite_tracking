"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr
import os

class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block - Save latest sample"""

    def __init__(self, filename="sample.txt"):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='Save latest sample to file',   # will show up in GRC
            in_sig=[np.float32],
            out_sig=[]
        )
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.filename = filename

    def work(self, input_items, output_items):
        file = open(".tmp","w")
        file.write(str(float(input_items[0][-1])))
        file.close()
        os.system(f"mv .tmp {os.path.expanduser('~/') + self.filename}")
        return len(input_items[0])
