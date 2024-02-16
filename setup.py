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
    install_requires=['setuptools', 'sounddevice', 'SpeechRecognition', 'elevenlabs', 'pygame'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Inputs
            'audio_collector.py = verbaliser.input_methods.audio_collector:main',
            # Dialogue
            'openai_chat.py = verbaliser.dialogue_methods.openai_chat:main',
            'openai_robotnav.py = verbaliser.dialogue_methods.openai_robotnav:main',
            # Outputs
            'speaker.py = verbaliser.output_methods.speaker:main',
            'elevenlabs_speaker.py = verbaliser.output_methods.elevenlabs_speaker:main',
            #Other
            'triggers.py = verbaliser.triggers:main'
        ],
    },

)

