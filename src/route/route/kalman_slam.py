import numpy as np


class KalmanSLAM:

  def __init__(self, state=np.array([0, 0, 1])) -> None:
    self.x = state
    self.P = np.zeros((self.x.shape[0], self.x.shape[0]))
    # keep the order of seen tags
    self.tag_ids = []
    self.fit_state()

  def fit_state(self):
    # fit matrices size to the state vector
    # state transform
    self.F = np.identity(self.x.shape[0])
    # control input
    self.G = np.identity(self.x.shape[0])
    # system & sensor noise
    self.Q = np.identity(self.x.shape[0]) * 0.1
    self.R = np.identity(self.x.shape[0]) * 0.1
    # Sigma: covariance matrix
    # keep the last covariance
    P_ = np.zeros((self.x.shape[0], self.x.shape[0]))
    P_[:self.P.shape[0], :self.P.shape[1]] = self.P
    self.P = P_
    # measurement matrix
    self.H = np.identity(self.x.shape[0])

  def predict(self, control):
    # state prediction
    self.x = self.F @ self.x + self.G @ control
    # error convariance prediction
    self.P = self.F @ self.P @ self.F.T + self.Q

  def update(self, measurement):
    unseen_tags = list(measurement['tags'].keys() - set(self.tag_ids))
    if len(unseen_tags) > 0:
      # expand the corresponding variables
      self.tag_ids += unseen_tags
      # state vector
      self.x = np.concatenate([self.x] + [measurement['tags'][tag_id] for tag_id in unseen_tags])
      self.fit_state()

    # assemble the measurement vector
    z = self.x.copy()
    z[:3] = measurement['bot']
    for tag_id, tag_m in measurement['tags'].items():
      idx = self.tag_ids.index(tag_id) + 1
      z[idx * 3:(idx + 1) * 3] = tag_m

    # perform update
    K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
    self.x = self.x + K @ (z - self.H @ self.x)
    self.P = (np.identity(K.shape[0]) - K @ self.H) @ self.P

  def step(self, control, measurement):
    '''
    control: [dx, dy, dr]
    measurement: {
      bot: [x, y, r],
      tags: {
        1: [x_t1, y_t1, r_t1],
        2: ...
      }
    }
    '''
    control_full = np.zeros_like(self.x)
    control_full[:3] = control
    self.predict(control_full)
    self.update(measurement)

  def get_tag_status(self):
    return {tid: self.x[(idx + 1) * 3:(idx + 2) * 3] for idx, tid in enumerate(self.tag_ids)}

  def get_bot_state(self):
    return self.x[:3]