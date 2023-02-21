# Hidden Markov Models

```
pip install tensorflow_probability --user --upgrade
```

```
import tensorflow_probability as tfp  # We are using a different module from tensorflow this time
import tensorflow as tf

tfd = tfp.distributions  # making a shortcut for later on
# Initially, 80% for the first state and 20% for the second state
initial_distribution = tfd.Categorical(probs=[0.8, 0.2])
# matrix: the first row (0.7 0.2) and the second row (0.3 0.8)
transition_distribution = tfd.Categorical(probs=[[0.7, 0.3],
                                                 [0.2, 0.8]])
# N(0, 5) and N(15, 10)
observation_distribution = tfd.Normal(loc=[0., 15.], scale=[5., 10.])

model = tfd.HiddenMarkovModel(
    initial_distribution=initial_distribution,
    transition_distribution=transition_distribution,
    observation_distribution=observation_distribution,
    num_steps=7)

mean = model.mean()
print(mean.numpy())
```
