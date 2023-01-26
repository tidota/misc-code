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


## Linear regression

### Simple linear regression

numpy has a feature for linear regression
```
import numpy as np
import matplotlib.pyplot as plt

x = [1, 2, 2.5, 3, 4]
y = [1, 4, 7, 9, 15]
plt.plot(x, y, 'ro')
plt.axis([0, 6, 0, 20])

plt.plot(x, y, 'ro')
plt.axis([0, 6, 0, 20])
params = np.polyfit(x, y, 1)
func = np.poly1d(params)
plt.plot(np.unique(x), func(np.unique(x)))
plt.show()
```

### More complicated linear regression

```
pip install -q sklearn
```

```
from __future__ import absolute_import, division, print_function, unicode_literals

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from IPython.display import clear_output
from six.moves import urllib

import tensorflow.compat.v2.feature_column as fc

import tensorflow as tf
```

Data loading
```
dftrain = pd.read_csv('https://storage.googleapis.com/tf-datasets/titanic/train.csv') # training data
dfeval = pd.read_csv('https://storage.googleapis.com/tf-datasets/titanic/eval.csv') # testing data
y_train = dftrain.pop('survived')
y_eval = dfeval.pop('survived')
```

Data inspection
```
dftrain.head(10)
dftrain.describe()
dftrain.shape
y_train.head()
dftrain.age.hist(bins=20)
dftrain.sex.value_counts().plot(kind='barh')
dftrain['class'].value_counts().plot(kind='barh')
pd.concat([dftrain, y_train], axis=1).groupby('sex').survived.mean().plot(kind='barh').set_xlabel('% survive')
```

Preparation of Inputs and the model
```
CATEGORICAL_COLUMNS = ['sex', 'n_siblings_spouses', 'parch', 'class', 'deck',
                       'embark_town', 'alone']
NUMERIC_COLUMNS = ['age', 'fare']

feature_columns = []
for feature_name in CATEGORICAL_COLUMNS:
  vocabulary = dftrain[feature_name].unique()  # gets a list of all unique values from given feature column
  feature_columns.append(tf.feature_column.categorical_column_with_vocabulary_list(feature_name, vocabulary))

for feature_name in NUMERIC_COLUMNS:
  feature_columns.append(tf.feature_column.numeric_column(feature_name, dtype=tf.float32))

linear_est = tf.estimator.LinearClassifier(feature_columns=feature_columns)
```


TODO: functions? and training?
