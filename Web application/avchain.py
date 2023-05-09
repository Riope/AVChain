#!/usr/bin/python3

from flask import Flask, render_template, url_for, redirect, flash, jsonify, request
import ipfshttpclient
import os
import subprocess
import time
import numpy as np
import json
import cv2
import ast

app = Flask('AV_Logchain')
app.config['SECRET_KEY'] = '5791628bb0b13ce0c676dfde280ba245'

@app.route('/avchain/')
@app.route('/avchain/home')
def home():
    return render_template('index.html')

@app.route("/avchain/simulate")
def invoke():
    return render_template('invoke.html', title='Simulate')

@app.route("/avchain/simulate/generate", methods=["GET", "POST"])
def generate():
    if request.method == "POST":
        try:
            town = request.form.get("town")
            vid = request.form.get("vehicleID")
            frames = request.form.get("frames")
            npcs = request.form.get("npcs")

            cmd = ["python3", "CARLA.py", "-r", "-s", "--invoke"]
            if town != '':
                cmd.extend(["--town", town])
            if vid != '':
                cmd.extend(["--vid", vid])
            if frames != '':
                cmd.extend(["--frames", frames])
            if npcs != '':
                cmd.extend(["--npcs", npcs])
            subprocess.run(cmd)

            flash('Simulation data generated!', 'success')
            return redirect(url_for('invoke'))
        except subprocess.CalledProcessError as e:
            return jsonify({"error": e.output.decode()}), 500
    return redirect(url_for('invoke'))

@app.route("/avchain/query")
def query():
    return render_template('query.html', title='Query')

@app.route("/avchain/query/ReadFrames", methods=["GET", "POST"])
def ReadVehicleFrames():
    if request.method == "POST":
        try:
            vid = request.form.get("vehicleID")
            cmd = ["python3", "CARLA.py", "--query", "ReadVehicleFrames", "--vid", vid]
            output = subprocess.check_output(cmd)

            json_out = json.loads(output)
            return render_template('output.html', title='Output', out=json_out['std_out'], cids=json_out['cids'])
        except subprocess.CalledProcessError as e:
            return jsonify({"error": e.output.decode()}), 500
    return redirect(url_for('query'))

@app.route("/avchain/query/ReadData", methods=["GET", "POST"])
def ReadFrameData():
    if request.method == "POST":
        try:
            vid = request.form.get("vehicleID")
            st_time = request.form.get("start_time")
            end_time = request.form.get("end_time")
            user = request.form.get("user")
            query_type = request.form.get("query_type")

            cmd = ["python3", "CARLA.py", "--user", user, "--query", query_type, "--vid", vid]
            if st_time != '':
                cmd.extend(["--start_time", st_time])
            if end_time != '':
                cmd.extend(["--end_time", end_time])
            output = subprocess.check_output(cmd)

            json_out = json.loads(output)
            for i in range(len(json_out['cids'])):
                json_out['cids'][i] = json_out['cids'][i].strip('\"')

            visual = False
            if query_type == 'ReadCam0Data' or query_type == 'ReadCam1Data' or query_type == 'ReadLIDARData':
                visual = True
            
            return render_template('output.html', title='Output', out=json_out['std_out'],\
                                        cids=json_out['cids'], visual=visual, active_link=True)
        except subprocess.CalledProcessError as e:
            return jsonify({"error": e.output.decode()}), 500
    return redirect(url_for('query'))

@app.route("/avchain/query/visualize", methods=["GET", "POST"])
def visualize():
    if request.method == "POST":
        cids_json = request.form.get("cids")
        cids = ast.literal_eval(cids_json)
        
        client = ipfshttpclient.connect('/ip4/127.0.0.1/tcp/5001/http')

        folder = 'static/output/'
        for the_file in os.listdir(folder):
            file_path = os.path.join(folder, the_file)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(e)

        i=1
        for cid in cids:
            data = json.loads(client.cat(cid).decode())
            cv2.imwrite("static/output/%03d.png" % i, np.array(data))
            cv2.waitKey(0)
            i = i+1
        
        return render_template('visual.html', title='Visualize', active_js=True)

    return render_template('visual.html', title='Visualize')

@app.route("/avchain/get_images")
def get_images():
    folder = os.path.join(os.getcwd(), "static", "output")
    image_filenames = os.listdir(folder)
    return jsonify(image_filenames)

if __name__ == "__main__":
    app.run(host='0.0.0.0',port=5001)
