import sys
import time 
import threading
from threading import Timer
import serial
import math
import matplotlib as mpl
import matplotlib.colors as colors
import matplotlib.pyplot as plt; plt.rcdefaults()
from matplotlib.font_manager import FontProperties
import numpy as np
import pylab

"""
    pyIM
    ====
    
    Script for displaying real time graphs of IM measurement

    :Date: 2014-08-12
    :Version: 1.0
    :Authors: Mathieu Girard

    Why this script?
    ----------------

    Keyence IM product is not able to handle display of gauges
    for measurement automatically.

    How does it work?
    -----------------

    This script uses serial communication. It looks for new
    measurement embedded a dedicated frame sent after each
    measurement.

    How to install?
    ---------------

    This script requires the following libraries:
        - Python 3.3
        - matplotlib
        - numpy
        - pyparsing
        - dateutil
        - six
        - pyserial
        
    Communication with IM works thanks to USB60F_Setup from RATOC Systems
    or you can use the standard RS232 serial communication port.

    License
    -------

    This software is under the GPL license
    Please look at: http://www.gnu.org/licenses/gpl-3.0.txt

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
    THE POSSIBILITY OF SUCH DAMAGE.
"""

class IM_Measure:
    """
        This class is used as a data structure for IM measurement
    """
    def __init__(self, design = 0, lower = 0, upper = 0, item = 0, unit = "", name = "", index = 0, type_index = 0):
        """
            IM_Measure constructor
            :design: nominal value
            :item: current value
            :upper: plus relative tolerance
            :lower: minus relative tolerance
            :unit: unit (eg.: mm, °, etc.)
            :name: name
            :index: index in the frame
        """        
        self.measure_lower = lower
        self.measure_upper = upper
        self.measure_design = design
        self.measure_item = item
        self.measure_unit = unit
        self.measure_name = name
        self.measure_index = index
        self.measure_type_index = type_index

    def __str__(self):
        return (("Name: %s\r\nDesign: %f\r\nLower: %f\r\nUpper: %f\r\nItem: %f\r\nUnit: %s\r\nIndex: %d\r\n")%(self.measure_name, self.measure_design, self.measure_lower, self.measure_upper, self.measure_item, self.measure_unit, self.measure_index))

    def computePerfMeasure(self):
        """
            Compute the relative gap in of the measurement
            :returns: relative deviation between [0.0 - 1.0]
        """   
        measure_range = self.measure_upper - self.measure_lower
        if measure_range != 0:
            nominal = ((self.measure_design + self.measure_upper) + (self.measure_design + self.measure_lower))/2
            return (2*(self.measure_item - nominal) / (measure_range))
        else:
            return 0

class IM_Frame:
    """
        IM_Frame is a class for storing frame information: header and measures
    """
    TYPE_NORMAL = 0
    TYPE_MULTI_PROG = 1
    TYPE_MULTI_PART = 2
        
    def __init__(self):
        """
            IM_Frame constructor
            :sn: IM serial number
            :ver: IM version
            :date: frame date
            :hour: frame hour
            :prog_name: program name according fram header
            :measures: array of IM_Measure
            :raw_frame: received frame as text
        """
        self.sn = ''
        self.ver = ''
        self.date = ''
        self.hour = ''
        self.prog_name = ''
        self.measures = []
        self.type = None
        self.type_index = 0
        self.raw_frames = []
        
    def __str__(self):
        printedStr = 'IM serialnumber: %s\r\nIM version: %s\r\nProgram name: %s\r\nExecution date: %s\r\nExecution hour: %s\r\n\r\n'%(self.sn,self.ver,self.prog_name,self.date,self.hour)
        for m in self.measures:
            printedStr = printedStr + str(m)
        return printedStr

    def checksum(self, raw_string):
        """
            This function compute the checksum of a frame line
            :returns: checksum hex value as string
        """           
        s = 0
        for i in range(0, len(raw_string)):
            s = s + ord(raw_string[i])
        hex_sum = format(s, 'X')
        return hex_sum[len(hex_sum)-2:len(hex_sum)-0]


    def parseFrame(self, raw_frame, check):
        """
            This function has to transform raw text to IM_Frame
            :returns: True if transformed correctly, False if not
        """            
        sep = '\t'
        frameOffset = 0
        type_index = 0

        # check frame checksum
        if (check):
            for line in raw_frame:
                if (self.checksum(line) != line[len(line)-2:len(line)-0]):
                    #print('Error: Incorrect checksum')
                    return False

        # check first line, if it is a start delimiter
        if (raw_frame[0][0:2] != 'ST'):
            return False

        # check if multiple program / part
        if (raw_frame[1][0:2] == 'JG'):
            self.type = IM_Frame.TYPE_MULTI_PART
            type_index = int(raw_frame[1].split(sep)[2])
            frameOffset = 1
        elif (raw_frame[1][0:2] == 'JH'):
            self.type = IM_Frame.TYPE_MULTI_PROG
            type_index = int(raw_frame[1].split(sep)[1])
            frameOffset = 1
        else:
            self.type = IM_Frame.TYPE_NORMAL
            type_index = 0

        # get sn and version of IM which are the same for a bunch of frames
        if (raw_frame[1+frameOffset][0:2] != 'SE'):
            return False
        else:
            self.sn, self.ver = raw_frame[1+frameOffset].split(sep)[1], raw_frame[1+frameOffset].split(sep)[2]

        # get date and time of frame
        if (raw_frame[2+frameOffset][0:2] != 'DA'):
            return False
        else:
            self.date, self.hour = raw_frame[2+frameOffset].split(sep)[1], raw_frame[2+frameOffset].split(sep)[2]

        # get program name
        if (raw_frame[3+frameOffset][0:2] != 'MS'):
            return False
        else:
            self.prog_name = raw_frame[3+frameOffset].split(sep)[1]

        # check LO
        if (raw_frame[4+frameOffset][0:2] != 'LO'):
            return False
        else:
            pass

        # check CH
        if (raw_frame[5+frameOffset][0:2] != 'CH'):
            return False
        else:
            pass

        # get all measures
        for i in range(6+frameOffset, len(raw_frame)-1):
            if (raw_frame[i][0:2] != 'IT'):
                return False
            else:
                strBlocks = raw_frame[i].split(sep)
                nameTemp = '#' + strBlocks[1] + ' '
                for j in range(4, len(strBlocks)-4):
                    nameTemp = nameTemp + ' ' + strBlocks[j]

                measure = IM_Measure(float(strBlocks[-5].replace(',', '.')), \
                                     float(strBlocks[-3].replace(',', '.')), \
                                     float(strBlocks[-4].replace(',', '.')), \
                                     float(strBlocks[2].replace(',', '.')), \
                                     strBlocks[3], nameTemp, int(strBlocks[1]), \
                                     type_index)
                self.measures.append(measure)

        return True

    def parseFrames(self, check):
        """
            This function has to get raw frames from general serial frame
            :returns: True if goten correctly, False if not
        """              
        self.raw_frames = []
        self.measures = []
        raw_frame = []

        for line in self.raw_frame:
            tmpStr = str(line, 'latin-1')
            raw_frame.append(tmpStr)
            # check if frame footer delimiter found
            if (tmpStr[0:2] == 'EN'):
                self.raw_frames.append(raw_frame)
                raw_frame = []

        # it will parse each single frame (for multi prog / part type)
        parseResult = True
        for f in self.raw_frames:
            parseResult = parseResult and self.parseFrame(f, check)
            
        return parseResult
            
        

def truncate_colormap(cmap, minval=0.0, maxval=1.0, n=100):
    """
        Reshape a color map to fit the performance of the measure
        based on http://stackoverflow.com/questions/18926031/how-to-extract-a-subset-of-a-colormap-as-a-new-colormap-in-matplotlib
        :returns: a reshaped matplotlib colormap
    """
    new_cmap = colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=minval, b=maxval),
        cmap(np.linspace(minval, maxval, n)))
    return new_cmap


class IM_Display:
    """
        Manage the display of the single figure with Matplotlib and
        polls periodically new frame from IM on the serial port
    """    
    def __init__(self, port, baud, check = False):
        """
            Initialize the main display: a single figure
            :port: serial port index or name (eg.: COM4)
            :paud: baud rate (eg.: 115200)
        """            
        self.frame = IM_Frame()
        self.new_frame = False
        self.lock = threading.Lock()
        self.check = check

        # disable figure toolbar, bind close event and open serial port
        with mpl.rc_context({'toolbar':False}): 
            self.fig = plt.figure()
            self.serial_port = serial.Serial(port, baud, timeout=0.25,bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, xonxoff=False)
            self.serial_port.flush()
            self.fig.canvas.mpl_connect('close_event', self.close_display)
            self.fig.canvas.mpl_connect('resize_event', self.resize_display)
            self.fig.canvas.set_window_title('pyIM')
            
            # window is prefered in maximzed state
            wm = plt.get_current_fig_manager()
            wm.window.state('zoomed')

            # create timer for updating display using a rs232 polling callback
            timer = self.fig.canvas.new_timer(interval=250)
            timer.add_callback(self.update_graphs) # then arg if needed
            timer.start()

            # launch figure
            plt.show(block = True)

            # close correctly serial port
            self.close_display()

    def close_display(self):
        """
            This functions should be called after closing the figure
        """         
        self.serial_port.close()
        
    def resize_display(self, event):
        """
            This functions is be called when resizeing the window
        """         
        plt.draw()      
    
    def update_graphs(self):
        """
            The figure is updated periodically thanks to the timer by this function
            http://matplotlib.org/examples/pylab_examples/gradient_bar.html
        """
        
        # start a thread for getting a new frame
        threadSerial = threading.Thread(target=self.get_frame, args=())
        threadSerial.start()

        # if no new frame, returns immediatelly
        if (self.new_frame != True):
            return

        # clear the previous axes and updates the window's title
        self.fig.clf()
        self.fig.canvas.set_window_title('%s %s %s'%(self.frame.prog_name, self.frame.date, self.frame.hour))
        mpl.rc('xtick', labelsize=10) 
        mpl.rc('ytick', labelsize=10)

        # compute the number of lines per screen (left and right have the same number)
        n_line_screen = round((len(self.frame.measures)%2 + len(self.frame.measures) + 4)/2)
        min_line_screen = 12
        if (n_line_screen < min_line_screen):
            n_line_screen = min_line_screen
        labels_left = ['']*n_line_screen
        labels_right = ['']*n_line_screen

        # initialize min/max percentage and graph number of lines
        boundary = 150
        xmin, xmax = xlim = -1 * boundary, boundary
        ymin, ymax = ylim = 0, n_line_screen-1
        
        # creates our dual screen layer
        ax_left = self.fig.add_subplot(121, xlim=xlim, ylim=ylim, autoscale_on=False)
        ax_right = self.fig.add_subplot(122, xlim=xlim, ylim=ylim, autoscale_on=False)

        # generate lines
        gradient = np.linspace(0, 1, 256)
        gradient = np.vstack((gradient, gradient))
        for i in range(0, len(self.frame.measures)):
            # adjust label regarding the measure type
            if (self.frame.type == IM_Frame.TYPE_NORMAL):
                measure_type = ''
            elif (self.frame.type == IM_Frame.TYPE_MULTI_PROG):
                measure_type = ('\nProgramme #%d'%self.frame.measures[i].measure_type_index)
            elif (self.frame.type == IM_Frame.TYPE_MULTI_PART):
                measure_type = ('\nObjet #%d'%self.frame.measures[i].measure_type_index)   
            #
            if (i < math.ceil(len(self.frame.measures)/2)): # math.floor(len(self.frame.measures)/2)
                ax = ax_left
                ypos = n_line_screen-i-2
                labels_left[ypos] = str('%s (%s)%s'%(self.frame.measures[i].measure_name, self.frame.measures[i].measure_unit, measure_type))
            else:
                ax = ax_right
                ypos = n_line_screen - i + math.ceil(len(self.frame.measures)/2) - 2
                labels_right[ypos] = str('%s (%s)%s'%(self.frame.measures[i].measure_name, self.frame.measures[i].measure_unit, measure_type))
                

            # set labels on each ax
            ax_left.set_yticklabels(labels_left)
            ax_right.set_yticklabels(labels_right)

            # calculation of coefficient for truncated colormap according ratio
            perf = self.frame.measures[i].computePerfMeasure()
            color_start = 0.5
            color_end = 0.85           
            cmap = plt.get_cmap('jet')
                
            # update color map according values then plot the image
            new_cmap = truncate_colormap(cmap, color_start, (color_end - color_start) * abs(perf) + color_start)
            ax.imshow(gradient, interpolation='bicubic', aspect='auto', cmap=new_cmap, extent=(0, perf*100, ypos-0.25, ypos+0.25 ))
 

            # setup text legend
            maxi = self.frame.measures[i].measure_design + self.frame.measures[i].measure_upper
            mini = self.frame.measures[i].measure_design + self.frame.measures[i].measure_lower
            if ((self.frame.measures[i].measure_item >= maxi) or (self.frame.measures[i].measure_item <= mini)):
                text_color = 'red'
            else:
                text_color = 'green'

            font_middle = FontProperties()
            font_middle.set_family('sans-serif')
            font_middle.set_weight('bold')

            font_side = FontProperties()
            font_side.set_family('sans-serif')
            font_side.set_weight('normal')
            
            alignment = {'horizontalalignment':'center', 'verticalalignment':'baseline'}

            ax.text(0, ypos, str('%.3f'%self.frame.measures[i].measure_item), color=text_color, fontproperties=font_middle,**alignment)
            ax.text(-100, ypos, str('%.3f'%mini), color='black', fontproperties=font_side,**alignment)
            ax.text(+100, ypos, str('%.3f'%maxi), color='black', fontproperties=font_side,**alignment)
            ax.axhline(linewidth=1, color='black', y=ypos, alpha = 0.05)

            
        # setup cosmetic for both axes
        for ax in [ax_left, ax_right]:  
            ax.set_aspect('auto')
            ax.set_aspect('auto')
            
            # set lighter axes
            for axes in ['top','right','bottom','left']:
                ax.spines[axes].set_linewidth(0.5)
                
            # keeps ticks only bottom and left side and set step 1
            ax.get_xaxis().tick_bottom()
            ax.get_yaxis().tick_left()
            start, end = ax.get_ylim()
            ax.yaxis.set_ticks(np.arange(start, end, 1))            
                        
            # set background color grey shade percentage
            ax.set_axis_bgcolor('0.85')

            # draw center vertical line as visual reference
            ax.axvline(linewidth=1, color='black', x=0, alpha = 0.05)
        
        # final layout setup and drawing
        plt.tight_layout()
        plt.draw()

        self.new_frame = False

    def get_frame(self):
        """
            Threaded function for getting a frame on serial line
        """
        self.lock.acquire()
        self.frame.raw_frame = self.serial_port.readlines()
        self.lock.release()
        if(self.frame.raw_frame != []):
            if (self.frame.parseFrames(self.check) == True):
                self.new_frame = True
                

if __name__ == '__main__':
    #IM_Display('com3', 9600, 0)
    if (len(sys.argv) < 3):
        print('Usage:\r\n\tpyIM.py port baudrate check')
    else:
        check = False
        if (len(sys.argv) == 4):
            if (sys.argv[3] == '1'):
                check = True
            else:
                check = False
        disp = IM_Display(sys.argv[1], sys.argv[2], check)
