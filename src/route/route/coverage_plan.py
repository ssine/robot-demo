# %%
import numpy as np
import matplotlib.pyplot as plt
import cv2
from functools import cache
import networkx as nx

plt.rcParams["figure.dpi"] = 200


# %%
def dist(p1, p2):
  return np.linalg.norm([p1[0] - p2[0], p1[1] - p2[1]])


def compress_waypoints(wps):
  '''
  remove consecutive points on the same line.
  '''
  idx = 0
  while True:
    if len(wps[idx:]) < 3:
      break
    if np.abs(dist(wps[idx + 1], wps[idx]) + dist(wps[idx + 2], wps[idx + 1]) - dist(wps[idx + 2], wps[idx])) < 1e-6:
      wps = wps[:idx + 1] + wps[idx + 2:]
    else:
      idx += 1
  return [list(wp) for wp in wps]


# %%
def coverage_plan_grid(border, bot_side_len, start_point, thresh=0.9, plot=False):
  # unit convert
  safe_len = bot_side_len
  bbox_irl = np.array([
      [np.min(border[:, 0]) - safe_len, np.min(border[:, 1]) - safe_len],
      [np.max(border[:, 0]) + safe_len, np.max(border[:, 1]) + safe_len],
  ])
  pixel_len = 0.01
  to_pix = lambda t: int(t / pixel_len)
  mp = np.zeros((to_pix(bbox_irl[1, 0] - bbox_irl[0, 0]), to_pix(bbox_irl[1, 1] - bbox_irl[0, 1])), dtype=np.float32)
  bot_px = to_pix(bot_side_len)
  border[:, 0] = (border[:, 0] - bbox_irl[0, 0]) / pixel_len
  border[:, 1] = (border[:, 1] - bbox_irl[0, 1]) / pixel_len
  border = border.astype(int)
  start_point = np.array([start_point[0] - bbox_irl[0, 0], start_point[1] - bbox_irl[0, 1]]) / pixel_len

  # generate map & bbox
  mp = cv2.fillPoly(mp, pts=[border], color=1)
  bbox = np.array([
      [np.min(border[:, 0]), np.min(border[:, 1])],
      [np.max(border[:, 0]), np.max(border[:, 1])],
  ])

  @cache
  def viable(x, y):
    return mp[x:x + bot_px, y:y + bot_px].sum() / (bot_px**2) > thresh

  # generate map
  num_y = int(np.ceil((bbox[1, 1] - bbox[0, 1]) / bot_px))
  g = nx.Graph()
  cnt = 0
  if plot:
    mmp = mp.copy()
  for x in range(bbox[0, 0], bbox[1, 0], bot_px):
    for y in range(bbox[0, 1], bbox[1, 1], bot_px):
      if not viable(x, y):
        cnt += 1
        continue
      # x and y inverted between cv & plt context
      g.add_node(cnt, y=x + bot_px / 2, x=y + bot_px / 2)
      if viable(x + bot_px, y):
        g.add_edge(cnt, cnt + num_y, length=1)
      if viable(x, y + bot_px):
        g.add_edge(cnt, cnt + 1, length=1)
      # slop edges make path messy
      # if viable(x + bot_px, y + bot_px):
      #   g.add_edge(cnt, cnt + 1 + num_y, length=np.sqrt(2))
      # if viable(x - bot_px, y + bot_px):
      #   g.add_edge(cnt, cnt + 1 - num_y, length=np.sqrt(2))
      if plot:
        cv2.rectangle(mmp, [y,x], [y+bot_px, x+bot_px], color=0.5, thickness=1)
      cnt += 1
  if plot:
    plt.imshow(mmp)
    for n in g.nodes:
      plt.scatter([g.nodes[n]['x']], [g.nodes[n]['y']], s=8)
    for n1, n2 in g.edges:
      plt.plot([g.nodes[n1]['x'], g.nodes[n2]['x']], [g.nodes[n1]['y'], g.nodes[n2]['y']], linewidth=1)
    plt.gca().invert_yaxis()
    plt.title('grid graph')
    plt.show()

  # find route
  p = nx.approximation.traveling_salesman_problem(g, cycle=False)
  # if plot:
  #   fig, ax = plt.subplots(figsize=(20, 20))
  #   edgelist = [[p[i], p[i + 1]] for i in range(len(p) - 1)]
  #   # nx.draw(g, pos=pos, ax=ax)
  #   pos = {i: (g.nodes[i]['x'], g.nodes[i]['y']) for i in g.nodes}
  #   nx.draw_networkx_edges(g, pos, edgelist, ax=ax, edge_color='r')
  #   plt.show()
  wps = np.array([(g.nodes[i]['x'], g.nodes[i]['y']) for i in p])
  # change start point & compress
  am = np.argmin(np.linalg.norm(wps - start_point, axis=1))
  wps = np.concatenate([[start_point], wps[am:], wps[:am]])
  wps = np.array(compress_waypoints(list(wps)))
  if plot:
    plt.imshow(mp)
    for n in g.nodes:
      plt.scatter([g.nodes[n]['x']], [g.nodes[n]['y']], s=8)
    plt.plot(wps[:, 0], wps[:, 1], linewidth=1)
    for idx, wp in enumerate(wps):
      plt.text(wp[0], wp[1], str(idx))
    plt.gca().invert_yaxis()
    plt.title('planned path')
    plt.show()

  # convert unit back
  wps = wps * pixel_len
  wps[:, 0] += bbox_irl[0, 0]
  wps[:, 1] += bbox_irl[0, 1]

  return wps


# %%
pts = coverage_plan_grid(
    np.array([
        [81, 49],
        [64, 103],
        [43, 216],
        [48, 303],
        [127, 335],
        [240, 344],
        [315, 314],
        [306, 224],
        [337, 128],
        [294, 66],
        [199, 50],
        [134, 47],
    ]) / 150, 0.2, [0.54, 0.32], plot=True)

# %%
pts
# %%
