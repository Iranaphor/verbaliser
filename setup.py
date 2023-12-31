from setuptools import setup

package_name = 'verbaliser'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'sounddevice', 'SpeechRecognition'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speaker.py = verbaliser.speaker:main',
            'audio_collector.py = verbaliser.audio_collector:main',
            'openai_chat.py = verbaliser.openai_chat:main',
            'openai_robotnav.py = verbaliser.openai_robotnav:main',
            'triggers.py = verbaliser.triggers:main'
        ],
    },

)

