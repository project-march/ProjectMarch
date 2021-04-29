from flask import Flask

app = Flask('march_mpc_visualizer')
app = Flask(__name__.split('.')[0])
