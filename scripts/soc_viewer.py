from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import functools
from absl import app
from dm_control import viewer
from dm_control.locomotion import soccer


def main(argv):
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')
  viewer.launch(environment_loader=functools.partial(soccer.load, team_size=2))


if __name__ == '__main__':
  app.run(main)

