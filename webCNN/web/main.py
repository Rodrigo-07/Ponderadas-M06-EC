from flask import Flask, request, render_template
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import os

app = Flask(__name__)

# Carregar o modelo
model_cnn = load_model('../modelo_mnist.h5')
model_linear = load_model('../modelo_mnist_linear.h5')

# Função para processar a imagem
def process_image(image, model):


    if model == "CNN":
        image = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
        image = cv2.resize(image, (28, 28))
        image = image / 255.0
        image = np.expand_dims(image, axis=-1)
        image = np.expand_dims(image, axis=0)

        return image
    elif model == "Linear":
        image = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
        image = cv2.resize(image, (28*28))
        image = image / 255.0
        image = np.expand_dims(image, axis=-1)
        image = np.expand_dims(image, axis=0)

        return image

def make_prediction(image, model):

    if model == "CNN":
        image = process_image(image, "CNN")
        prediction = model_cnn.predict(image)
        print(prediction)

        return np.argmax(prediction)
    elif model == "Linear":
        image = process_image(image, "Linear")
        prediction = model_linear.predict(image)
        print(prediction)

        return np.argmax(prediction)

def get_class_name(class_id):
    classes = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0"]
    return classes[class_id]


@app.route('/')
def hello_world():
    return render_template('index.html')

@app.route('/predictCNN/', methods=['POST'])
def predictCNN():
    # Pegar imagem do formulário
    image = request.files['image']
    
    # Salvar a imagem
    image.save('image.png')

    prediction = make_prediction('image.png', "CNN")
    class_name = get_class_name(prediction)
    print(class_name)

    # Excluir a imagem
    os.remove('image.png')

    return render_template('index.html', predictionCNN=class_name)

@app.route('/predictLinear/', methods=['POST'])
def predictLinear():
    # Pegar imagem do formulário
    image = request.files['image']
    
    # Salvar a imagem
    image.save('image.png')


    prediction = make_prediction('image.png', "Linear")
    class_name = get_class_name(prediction)
    print(class_name)

    # Excluir a imagem
    os.remove('image.png')

    return render_template('index.html', predictionLinear=class_name)

if __name__ == '__main__':
    app.run()
