import numpy as np
import time
import pickle


class KalmanSLAM:

  def __init__(self, state) -> None:
    self.x = state
    self.P = np.zeros((self.x.shape[0], self.x.shape[0]))
    # keep the order of seen tags
    self.tag_ids = []
    self.fit_state()
    self.history = []
    self.save_state()

  def fit_state(self):
    # fit matrices size to the state vector
    # state transform
    self.F = np.identity(self.x.shape[0])
    # control input
    self.G = np.identity(self.x.shape[0])
    # system & sensor noise
    self.Q = np.identity(self.x.shape[0]) * 0.05
    self.R = np.identity(self.x.shape[0]) * 0.3
    # Sigma: covariance matrix
    # keep the last covariance
    P_ = np.zeros((self.x.shape[0], self.x.shape[0]))
    P_[:self.P.shape[0], :self.P.shape[1]] = self.P
    self.P = P_
    # measurement matrix
    self.H = np.zeros((len(self.tags_id) * 3, self.s.shape[0]))
    th = self.s[2]
    H_b = np.array([[-np.cos(th), -np.sin(th), 0], [np.sin(th), -np.cos(th), 0], [0, 0, -1]])
    H_t = np.array([[np.cos(th), np.sin(th), 0], [-np.sin(th), np.cos(th), 0], [0, 0, 1]])
    self.H[:3, :3] = H_b
    for idx in range(1, len(self.tags_id)):
      self.H[idx * 3:(idx + 1) * 3, idx * 3:(idx + 1) * 3] = H_t

  def predict(self, control):
    # state prediction
    self.x = self.F @ self.x + self.G @ control
    # error convariance prediction
    self.P = self.F @ self.P @ self.F.T + self.Q

  def update(self, measurement):
    '''
    measurement: {
      tags: {
        1: [x_t1, y_t1, r_t1],
        2: ...
      }
    }
    '''
    unseen_tags = list(measurement['tags'].keys() - set(self.tag_ids))
    if len(unseen_tags) > 0:
      # expand the corresponding variables
      self.tag_ids += unseen_tags
      # state vector
      self.x = np.concatenate([self.x] + [self.map_tag(measurement['tags'][tag_id]) for tag_id in unseen_tags])
      self.fit_state()

    # assemble the measurement vector
    z = np.zeros(len(self.tag_ids) * 3)
    for tag_id, tag_m in measurement['tags'].items():
      idx = self.tag_ids.index(tag_id)
      z[idx * 3:(idx + 1) * 3] = tag_m

    # perform update
    K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
    self.x = self.x + K @ (z - self.H @ self.x)
    self.P = (np.identity(K.shape[0]) - K @ self.H) @ self.P
    self.save_state()

  def step(self, control):
    '''
    control: [dx, dy, dr]
    '''
    control_full = np.zeros_like(self.x)
    control_full[:3] = control
    self.predict(control_full)
    self.save_state()

  def get_tag_status(self):
    return {tid: self.x[(idx + 1) * 3:(idx + 2) * 3] for idx, tid in enumerate(self.tag_ids)}

  def get_bot_state(self):
    return self.x[:3]

  def save_state(self):
    self.history.append({'x': self.x.copy(), 'P': self.P.copy(), 'tag_ids': [*self.tag_ids], 'time': time.time()})

  def export_state(self, path):
    with open(path, 'wb') as f:
      pickle.dump(self.history, f)

  def map_tag(self, tag):
    bot_x, bot_y, bot_theta = self.s[:3]
    tag_x_b, tag_y_b, tag_theta_b = tag
    tag_x_m = bot_x + tag_x_b * np.cos(bot_theta) - tag_y_b * np.sin(bot_theta)
    tag_y_m = bot_y + tag_x_b * np.sin(bot_theta) + tag_y_b * np.cos(bot_theta)
    tag_theta_m = bot_theta + tag_theta_b
    return np.array([tag_x_m, tag_y_m, tag_theta_m])
