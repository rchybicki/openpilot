from flask import Flask, render_template, request, send_from_directory
import os
from common.params import Params
import datetime
params = Params()
keys = params.all_keys()

dir_path = os.path.dirname(os.path.realpath(__file__))
SEGMENTS_DIR = "/data/media/0/realdata"
#SEGMENTS_DIR = os.path.join(dir_path, "video")

app = Flask(__name__)

@app.route("/")
def index():
  return render_template('pages/index.html')

@app.route("/params")
def params_page():
  return render_template('pages/params.html')

@app.route("/videos")
def video_page():
  dirs = [ "--".join(dir.split("--")[:-1]) for dir in os.listdir(SEGMENTS_DIR) if dir.find("--") >= 0 ]
  dirs = set(dirs)
  dirs = list(dirs)
  dirs.sort()
  return render_template('pages/videos.html', options=dirs)

@app.route("/route.html")
def route_partial():
  route = request.args.get('route', '')
  return render_template('route/route.html', route=route)

def sortSegments(segment):
  return int(segment.split("--")[-1])

@app.route("/route/<route>/playlist.m3u8")
def playlist_partial(route):
  dirs = [ dir for dir in os.listdir(SEGMENTS_DIR) if dir.startswith(route) ]
  dirs.sort(key=sortSegments)
  idx = 0
  segments = []
  for dir in dirs:
    probe = os.popen(f'ffprobe -i {SEGMENTS_DIR}/{dir}/qcamera.ts -show_entries format=duration -v quiet -of csv="p=0"')
    duration = float(probe.read().strip())
    duration = "{:.3f}".format(duration)
    segments.append((idx, duration, f"/video/{dir}/qcamera.ts"))
    idx+=1
  return render_template('route/playlist.m3u8', segments=segments)

@app.route("/video.html")
def video_partial():
  route = request.args.get('route', '')
  return render_template('video/video.html', route=route)

@app.route("/params.html")
def params_partial():
  parameters = [(key.decode('utf-8'), str(params.get(key))) for key in keys]
  return render_template('params/params.html', params=parameters)

@app.route("/route/segments.html")
def segments_partial():
  route = request.args.get('route', '')
  dirs = [ dir for dir in os.listdir(SEGMENTS_DIR) if dir.startswith(route) ]
  dirs.sort(key=sortSegments)
  idx = 0
  segments = []
  t = 0
  for dir in dirs:
    probe = os.popen(f'ffprobe -i {SEGMENTS_DIR}/{dir}/qcamera.ts -show_entries format=duration -v quiet -of csv="p=0"')
    duration = float(probe.read().strip())
    links = os.listdir(os.path.join(SEGMENTS_DIR, dir))
    links = [(dir, link) for link in links]
    segments.append({
      "name": dir,
      "time": str(datetime.timedelta(seconds=t)),
      "links": links
    })
    t += duration
    idx+=1
  return render_template('route/segments.html', segments=segments)

@app.route('/video/<path:path>')
def send_report(path):
    return send_from_directory(SEGMENTS_DIR, path, as_attachment=True)

def main():
  app.run(host="0.0.0.0", port=5050)

if __name__ == '__main__':
  main()
