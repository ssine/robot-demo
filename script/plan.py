# %%
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
import cv2
import networkx as nx

# %%
map_side_len = 0.3048 * 10
obstacle_side_len = 0.3048 * 2
robot_side_len = 0.2
pixel_len = 0.01

d = (map_side_len - obstacle_side_len) / 2 / 2
start_pos = map_side_len - d, d
stop_pos = d, map_side_len - d

# %%
to_pix = lambda t: int(t / pixel_len)
side_pix = to_pix(map_side_len)
mp = np.ones((side_pix, side_pix), dtype=np.float32)
st = to_pix(d * 2)
ed = st + to_pix(obstacle_side_len)
mp[st:ed, st:ed] = 0
mp[:, :1] = 0
mp[:, -1:] = 0
mp[:1, :] = 0
mp[-1:, :] = 0

# %%
plt.imshow(mp)
plt.gca().invert_yaxis()
plt.title('the map')

# %%
bot_px = to_pix(robot_side_len)
cfg_map = cv2.erode(mp, np.ones((bot_px, bot_px)))
plt.imshow(cfg_map)
plt.gca().invert_yaxis()
plt.title('configuration space')

# %%
skeleton = skeletonize(cfg_map)
plt.imshow(skeleton)
plt.scatter([to_pix(start_pos[0]), to_pix(stop_pos[0])], [to_pix(start_pos[1]), to_pix(stop_pos[1])])
plt.gca().invert_yaxis()
plt.title('medial axis transform')

# %%
xs, ys = np.where(skeleton)
xs = np.concatenate(([to_pix(start_pos[0])], xs, [to_pix(stop_pos[0])]))
ys = np.concatenate(([to_pix(start_pos[1])], ys, [to_pix(stop_pos[1])]))

g = nx.Graph()

n = xs.shape[0]
g.add_nodes_from(range(n))

for i in range(n):
  dis = np.empty(n)
  for j in range(n):
    dis[j] = np.sqrt((xs[i] - xs[j])**2 + (ys[i] - ys[j])**2)
  dis[i] = np.Infinity
  idxs, = np.where(dis - np.min(dis) < 1)
  for idx in idxs:
    g.add_edge(i, idx)

# %%
p = nx.shortest_path(g, 0, n - 1)
wps = np.array(list(zip(xs[p], ys[p]))) * pixel_len


# %%
def compress_waypoints(wps):
  '''
  remove consecutive points on the same line.
  '''
  idx = 0
  while True:
    if len(wps[idx:]) < 3:
      break
    d0 = wps[idx + 1] - wps[idx]
    d1 = wps[idx + 2] - wps[idx + 1]
    same_line = False
    if d0[0] == 0 or d1[0] == 0:
      if d1[0] == d0[0]:
        same_line = True
      else:
        idx += 1
        continue
    if not same_line:
      if np.abs(d1[1] / d1[0] - d0[1] / d0[0]) < 0.0001:
        same_line = True
    if same_line:
      wps = wps[:idx + 1] + wps[idx + 2:]
    else:
      idx += 1
  return wps


def draw_wps(wps, title):
  wps = np.array(wps)
  plt.imshow(cfg_map)
  plt.gca().invert_yaxis()
  plt.plot(wps[:, 0] / pixel_len, wps[:, 1] / pixel_len)
  plt.title(title)


# %%
draw_wps(compress_waypoints(list(wps)), 'maximum safety path')

# %%
plt.imshow(cv2.cornerHarris(cfg_map, 2, 3, 0.04))
plt.title('harris corner detection')
# %%
xs, ys = np.where(cv2.cornerHarris(cfg_map, 2, 3, 0.04) > 0.1)
xs = np.concatenate(([to_pix(start_pos[0])], xs, [to_pix(stop_pos[0])]))
ys = np.concatenate(([to_pix(start_pos[1])], ys, [to_pix(stop_pos[1])]))

g = nx.Graph()

n = xs.shape[0]
g.add_nodes_from(range(n))
for i in range(n):
  for j in range(n):
    if i == j: continue
    path = np.linspace([xs[i], ys[i]], [xs[j], ys[j]], 20).astype(int)[1:-1]
    if np.all(cfg_map[path[:, 0], path[:, 1]] > 0.5) or np.all(cfg_map[path[:, 0] - 3, path[:, 1] - 3] > 0.5) or np.all(
        cfg_map[path[:, 0] + 3, path[:, 1] + 3] > 0.5):
      g.add_edge(i, j, weight=np.sqrt((xs[i] - xs[j])**2 + (ys[i] - ys[j])**2))

p = nx.shortest_path(g, 0, n - 1, weight='weight')
wps = np.array(list(zip(xs[p], ys[p]))) * pixel_len

# %%
nx.draw(g, pos={i: (xs[i], ys[i]) for i in range(n)})
plt.title('corner path')

# %%
draw_wps(wps, 'minimum distance path')

# %%
