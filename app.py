from flask import Flask, render_template
import logging

app = Flask(__name__)

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    print("🚀 السيرفر يعمل على http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False)
