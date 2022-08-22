from distutils.core import setup

setup(name='calibrator_helper',
      version='0.01',
      description='Helper python scripts for camera calibration',
      author='Matti Jukola',
      author_email='buq2@buq2.com',
      packages=['calibrator_helper'],
      package_dir={'calibrator_helper': 'src/calibrator_helper'},
)
