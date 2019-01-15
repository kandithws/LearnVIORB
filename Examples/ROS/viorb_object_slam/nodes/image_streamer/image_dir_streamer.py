from threading import Thread, Condition, Lock

try: 
    from Queue import Queue, Full, Empty
except ImportError:
    from queue import Queue, Full, Empty

from PIL import Image
import time
from datetime import datetime
from os import listdir, mkdir
from os.path import isfile, join, splitext, isdir, basename
import re
import argparse
# import cv2
import numpy as np  


class ImageDirStreamer:
    def __init__(self, input_dir, rate=20, queue_size=1):
        self.shutdown = True
        self._stream_thread = Thread(target=self.stream_image_dir_worker)
        # self._stream_thread.daemon = True
        self._duration = 1.0/rate #ms
        self._img_queue = Queue(maxsize=queue_size) # Mutex already managed here
        img_format = re.compile('.(jpeg|jpg|png)')
        image_dir = [ f for f in listdir(input_dir) if isfile(join(input_dir, f)) and bool(img_format.match(splitext(join(input_dir, f))[1])) ]
        print("Total Image count %d" % (len(image_dir)))
        image_dir.sort()
        self._image_dir = [ join(input_dir, f) for f in image_dir]
        self._last_stamp = None
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, *args):
        self.stop()
    
    def is_shutdown(self):
        return self.shutdown

    def start(self):
        self.shutdown = False
        self._stream_thread.start()

    def stop(self):
        if not self.shutdown:
            self.shutdown = True
            self._stream_thread.join(timeout=2)
 

    def __del__(self):
        self.stop()

    def stream_image_dir_worker(self):
        for img_file in self._image_dir:
            if self.shutdown:
                break
            img = np.array(Image.open(img_file))
            stamp = int(splitext(basename(img_file))[0])
            try:
                self._img_queue.put_nowait( (img, stamp) )
            except Full:
                try:
                    # Sometimes get image worker thread could pop before we pop
                    _ = self._img_queue.get_nowait() # pop
                    self._img_queue.put_nowait( (img, stamp) )
                    #self._img_queue.task_done()
                    # self._img_queue.put_nowait( (img, stamp) )
                except Empty:
                    self._img_queue.put_nowait( (img, stamp) )
                # self._img_queue.all_tasks_done()
            
            time.sleep(self._duration) #simulate realtime
        # in case streaming is done
        self.shutdown = True
        print("DONE!!!")

    def get_image(self):
        if not self.shutdown:
            try:
                img, stamp = self._img_queue.get(True) # automatically wait
                self._img_queue.task_done() # here because we want to perform real-time streaming
                if self._last_stamp is not None:
                    print("Frame Streaming Diff: {} s".format(self.get_time_diff_from_stamp(stamp, self._last_stamp)))    
                self._last_stamp = stamp
                return img
            except Empty:
                return None
        else:
            return None
    
    def get_time_diff_from_stamp(self, t1, t2):
        assert(t1 > t2)
        dt1 = datetime.fromtimestamp(t1 / 1e9)
        dt2 = datetime.fromtimestamp(t2 / 1e9)
        return (dt1 - dt2).total_seconds()



if __name__ == '__main__':
    #testing 
    import cv2
    import numpy as np  
    # global outim  
    parser = argparse.ArgumentParser(description='Test Image Streamer')
    parser.add_argument('--inputdir',dest='input_dir', help='Input directory for streaming', required=True)
    parser.add_argument('--rate', dest='rate', type=int, default=20, help='Streaming rate')
    parser.add_argument('--qsize', dest='qsize', type=int,default=10, help='Image Buffer Size')
    args = parser.parse_args()
    streamer = ImageDirStreamer(args.input_dir, rate=args.rate, queue_size=args.qsize)
    streamer.start()
    try:
        while not streamer.shutdown:
            # streamer.get_image()
            img = streamer.get_image()
            if img:
                cv_image = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR) 
                cv2.imshow('image', cv_image)
                cv2.waitKey(1)
        cv2.destroyAllWindows()
        streamer.stop()
    except KeyboardInterrupt:
        streamer.stop()
        cv2.destroyAllWindows()
    cv2.destroyAllWindows()

