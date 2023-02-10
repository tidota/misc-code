# Linear regression

# Simple linear regression

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

-------------------------------------------------------------------------

# More complicated linear regression by using tensorflow

This looks like just classification? But unlike the one in [the page](./classification.md), this also handles categorical values.

# Setup
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

# Load data
```
dftrain = pd.read_csv('https://storage.googleapis.com/tf-datasets/titanic/train.csv') # training data
dfeval = pd.read_csv('https://storage.googleapis.com/tf-datasets/titanic/eval.csv') # testing data
y_train = dftrain.pop('survived')
y_eval = dfeval.pop('survived')
```

# Inspect data
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

# Create a classifier
Note that this step requires the input data for categorical clumns.
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

# Convert the inputs to the `tf.data.Dataset` object
```
def make_input_fn(data_df, label_df, num_epochs=10, shuffle=True, batch_size=32):
  def input_function():  # inner function, this will be returned
    ds = tf.data.Dataset.from_tensor_slices((dict(data_df), label_df))  # create tf.data.Dataset object with data and its label
    if shuffle:
      ds = ds.shuffle(1000)  # randomize order of data
    ds = ds.batch(batch_size).repeat(num_epochs)  # split dataset into batches of 32 and repeat process for number of epochs
    return ds  # return a batch of the dataset
  return input_function  # return a function object for use

train_input_fn = make_input_fn(dftrain, y_train)  # here we will call the input_function that was returned to us to get a dataset object we can feed to the model
eval_input_fn = make_input_fn(dfeval, y_eval, num_epochs=1, shuffle=False)
```

# Training
```
linear_est.train(train_input_fn)  # train
```

# Testing
```
result = linear_est.evaluate(eval_input_fn)  # get model metrics/stats by testing on tetsing data

clear_output()  # clears console output
print(result)
print(result['accuracy'])  # the result variable is simply a dict of stats about our model
```

# Evaluation
```
pred_dicts = list(linear_est.predict(eval_input_fn))
print(pred_dicts)
probs = pd.Series([pred['probabilities'][1] for pred in pred_dicts])

probs.plot(kind='hist', bins=20, title='predicted probabilities')
```
