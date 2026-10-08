"""Small CPU behavior-cloning classifier: 7 inputs, 32 hidden units, two action heads."""
import argparse
import json
import hashlib
from pathlib import Path
import numpy as np
from pose_prepare import FORWARD_LEVELS, TURN_LEVELS, load_dataset
from pose_task import FEATURE_NAMES


class Policy:
    def __init__(self, mean, scale, weights):
        self.mean, self.scale, self.weights = mean, scale, weights

    def probabilities(self, x):
        hidden = np.tanh(((np.asarray(x)-self.mean)/self.scale) @ self.weights['w1'] + self.weights['b1'])
        logits = (hidden @ self.weights['w2'] + self.weights['b2']).reshape(-1, 2, 5)
        exp = np.exp(logits-logits.max(axis=2, keepdims=True))
        return exp/exp.sum(axis=2, keepdims=True)

    def action(self, observation):
        classes = self.probabilities(np.asarray(observation)[None, :])[0].argmax(axis=1)
        return float(FORWARD_LEVELS[classes[0]]), float(TURN_LEVELS[classes[1]]), 0.0

    def save(self, path, metadata):
        path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('xb') as handle:
            np.savez_compressed(handle, mean=self.mean, scale=self.scale, **self.weights,
                                metadata=np.array(json.dumps(metadata, ensure_ascii=False)))

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            metadata = json.loads(str(data['metadata']))
            if metadata.get('schema') != 'pose-bc-v1' or metadata.get('feature_names') != FEATURE_NAMES:
                raise ValueError('Unsupported policy format')
            values = {key: data[key].copy() for key in ('mean', 'scale', 'w1', 'b1', 'w2', 'b2')}
        shapes = {'mean': (7,), 'scale': (7,), 'w1': (7,32), 'b1': (32,), 'w2': (32,10), 'b2': (10,)}
        if any(values[k].shape != shape or not np.isfinite(values[k]).all() for k, shape in shapes.items()) or (values['scale'] <= 0).any():
            raise ValueError('Invalid policy weights or scaling')
        return cls(values.pop('mean'), values.pop('scale'), values), metadata


def loss_and_gradient(policy, x, labels):
    normalized = (x-policy.mean)/policy.scale
    hidden = np.tanh(normalized @ policy.weights['w1']+policy.weights['b1'])
    p = policy.probabilities(x)
    n = len(x)
    selected = p[np.arange(n)[:,None], np.arange(2)[None,:], labels]
    loss = float(-np.log(np.maximum(selected, 1e-12)).mean())
    gradient = p.copy()
    gradient[np.arange(n)[:,None], np.arange(2)[None,:], labels] -= 1
    gradient = gradient.reshape(n, 10)/(2*n)
    delta = (gradient @ policy.weights['w2'].T)*(1-hidden**2)
    return loss, {'w2': hidden.T @ gradient, 'b2': gradient.sum(axis=0),
                  'w1': normalized.T @ delta, 'b1': delta.sum(axis=0)}


def train_arrays(train_x, train_y, val_x, val_y, epochs=100, seed=1729):
    if not 1 <= epochs <= 1000: raise ValueError('Epochs must be in 1..1000')
    rng = np.random.default_rng(seed)
    mean = train_x.mean(axis=0); scale = train_x.std(axis=0)
    scale[scale < 1e-3] = 1.0
    weights = {'w1': rng.normal(0, .2, (7,32)), 'b1': np.zeros(32),
               'w2': rng.normal(0, .15, (32,10)), 'b2': np.zeros(10)}
    policy = Policy(mean, scale, weights)
    first = loss_and_gradient(policy, val_x, val_y)[0]
    best, best_loss, stale, step = None, float('inf'), 0, 0
    moments = {k: np.zeros_like(v) for k, v in weights.items()}
    variances = {k: np.zeros_like(v) for k, v in weights.items()}
    history = []
    for epoch in range(epochs):
        indices = rng.permutation(len(train_x))
        for offset in range(0, len(indices), 128):
            batch = indices[offset:offset+128]
            _, grads = loss_and_gradient(policy, train_x[batch], train_y[batch])
            step += 1
            for key, grad in grads.items():
                moments[key] = .9*moments[key]+.1*grad
                variances[key] = .999*variances[key]+.001*grad**2
                weights[key] -= .003*(moments[key]/(1-.9**step))/(np.sqrt(variances[key]/(1-.999**step))+1e-8)
        val_loss = loss_and_gradient(policy, val_x, val_y)[0]
        history.append({'epoch': epoch+1, 'validation_cross_entropy': val_loss})
        if val_loss < best_loss-1e-5:
            best_loss = val_loss; best = {k: v.copy() for k,v in weights.items()}; stale = 0
        else:
            stale += 1
            if stale >= 12: break
    policy.weights = best
    accuracy = float((policy.probabilities(val_x).argmax(axis=2) == val_y).all(axis=1).mean())
    return policy, {'initial_validation_loss': first, 'best_validation_loss': best_loss,
                    'validation_joint_action_accuracy': accuracy, 'history': history,
                    'note': 'Action agreement is not closed-loop robot success.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--epochs', type=int, default=100)
    args = parser.parse_args()
    if args.output.exists(): parser.error('Choose a new model output path')
    arrays, manifest = load_dataset(args.dataset)
    policy, report = train_arrays(arrays['train_x'], arrays['train_y'], arrays['val_x'], arrays['val_y'], args.epochs)
    metadata = {'schema': 'pose-bc-v1', 'feature_names': FEATURE_NAMES,
                'training_source': 'human_pose_demos', 'dataset_sha256': hashlib.sha256(args.dataset.read_bytes()).hexdigest(),
                'dataset_manifest': manifest, 'training_report': report,
                'architecture': '7-32-tanh-two-5-class-heads', 'seed': 1729}
    policy.save(args.output, metadata)
    print(json.dumps({k:v for k,v in report.items() if k != 'history'}, indent=2))


if __name__ == '__main__': main()
