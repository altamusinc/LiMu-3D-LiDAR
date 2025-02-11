from flask import Flask, render_template, send_from_directory

webAPI = Flask(__name__)

@webAPI.route("/")
def serveHomepage():
    return render_template("index2.html")

@webAPI.route("/<path:filename>")
def serveFile(filename):
    return send_from_directory("./web", filename)

if __name__ == '__main__':
    webAPI.run(debug=True)
