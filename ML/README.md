# notes of ML practice

Disclaimer: This is my personal notes about machine learning stuff. While my background covers basics of machine learning, it lacked knowledge of the modern deep learning stuff. As I am personally just following some online courses, there may be (a lot of) errors and confusion in my notes. (so it may not be useful for others at all)

Codecamp's lectures
https://www.freecodecamp.org/learn/machine-learning-with-python/#tensorflow

Youtube video with links to Colab notebooks
https://www.youtube.com/watch?v=tPYj3fFJGjk&ab_channel=freeCodeCamp.org

- [Module 2: Introduction to TensorFlow](https://colab.research.google.com/drive/1F_EWVKa8rbMXi3_fG0w7AtcscFq7Hi7B#forceEdit=true&sandboxMode=true)
- [Module 3: Core Learning Algorithms](https://colab.research.google.com/drive/15Cyy2H7nT40sGR7TBN5wBvgTd57mVKay#forceEdit=true&sandboxMode=true)
- [Module 4: Neural Networks with TensorFlow](https://colab.research.google.com/drive/1m2cg3D1x3j5vrFc-Cu0gMvc48gWyCOuG#forceEdit=true&sandboxMode=true)
- [Module 5: Deep Computer Vision](https://colab.research.google.com/drive/1ZZXnCjFEOkp_KdNcNabd14yok0BAIuwS#forceEdit=true&sandboxMode=true)
- [Module 6: Natural Language Processing with RNNs](https://colab.research.google.com/drive/1ysEKrw_LE2jMndo1snrZUh5w87LQsCxk#forceEdit=true&sandboxMode=true)
- [Module 7: Reinforcement Learning](https://colab.research.google.com/drive/1IlrlS3bB8t1Gd5Pogol4MIwUxlAjhWOQ#forceEdit=true&sandboxMode=true)

I copied the Colab notebooks and tried to run the code for practice. The rest of this page is personal notes of each Colab notebook or just tables of contents in the other mark-down pages.

# Module 2 (Tensorflow)

Installation
```
pip install tensorflow
pip install tensorflow-gpu
```

Import
```
import tensorflow as tf
```

Data type
- `tf.string`
- `tf.int16`
- `tf.float64`
etc

Creating data
```
tns = tf.Variable([["abc", "def"], ["ghi", "jkl"]], tf.string)
tns2 = tf.ones([1,2,3])
tns3 = tf.reshape(tns2, [2, 3])
```

Information about a tensor
```
tf.rank(tns)     # shows the rank and data type
tns.shape        # shows the numbers of items at each dimension
len(tns.shape)   # gives the rank number
```

# Module 3 (Core learning algorithms)

## [Linear Regression](./module3/linear_regression.md)
The example also contains categorial inputs to give a continuous value as output.
It uses `sklearn`.

## [Classification](./module3/classification.md)
Categorial inputs to categorial output.

## [Hidden Markov Models](./module3/hidden_markov_models.md)
Continuous values as input to continuous value as output.
Or more like a probability distribution to a probability distribution.

# Module 4 (Neural Network)

Keras: submodule of tensorflow to provide neural network APIs.

## Preparation
```
import tensorflow as tf
from tensorflow import keras
import numpy as np
import matplotlib.pyplot as plt

fashion_mnist = keras.datasets.fashion_mnist
(train_images, train_labels), (test_images, test_labels) = fashion_mnist.load_data()

class_names = ['T-shirt/top', 'Trouser', 'Pullover', 'Dress', 'Coat',
               'Sandal', 'Shirt', 'Sneaker', 'Bag', 'Ankle boot']

train_images = train_images / 255.0
test_images = test_images / 255.0
```

## Inspection
```
train_images.shape
train_images[0,23,23]
train_labels[:10]

plt.figure()
plt.imshow(train_images[0])
plt.colorbar()
plt.grid(False)
plt.show()
```

## Building, compiling, and training the model
```
model = keras.Sequential([
    keras.layers.Flatten(input_shape=(28, 28)),
    keras.layers.Dense(128, activation='relu'),
    keras.layers.Dense(10, activation='softmax')
])

model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

model.fit(train_images, train_labels, epochs=10)
```

## Evaluation
```
test_loss, test_acc = model.evaluate(test_images,  test_labels, verbose=1)
print('Test accuracy:', test_acc)
```

## Prediction
```
predictions = model.predict(test_images)
np.argmax(predictions[0])  # should be the same as test_labels[0]
```

# Module 5 (Deep Computer Vision)

Convolutional Neural Network

Just a list of words
- Feature maps -> Response maps
- Pooling: downsampling and resizing layers
- Conv -> Dense
- Data Augmentation
- Pretrained models with fine tuning
