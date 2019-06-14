import os
import sys
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from sphinx import cmdline


def build_file(fname):
    print("Building {}".format(fname))
    doc_dir = os.getcwd()
    src_dir = os.path.join(doc_dir, "")
    html_dir = os.path.join(doc_dir, "build/html")
    cmdline.main([src_dir, html_dir, fname])


class Handler(FileSystemEventHandler):
    
    def on_modified(self, event):
        if event.is_directory:
            return
        fname = os.path.basename(event.src_path)
        if os.path.splitext(fname)[1].lower() == ".rst":
            print("{0}: {1}".format(event.event_type, fname))
            build_file(event.src_path)


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else '.'
    event_handler = Handler()
    observer = Observer()
    observer.schedule(event_handler, path, recursive=True)
    observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
observer.join()

