import os
from ament_index_python import get_package_share_directory
# from sound_play.libsoundplay import SoundClient


class Sounds(object):
    def __init__(self, sound_names=None, node=None):
        # self._sound_client = SoundClient()
        self._node = node
        self._sounds = {}
        self._sounds_dir = os.path.join(get_package_share_directory('march_gait_selection'), 'sounds')

        if sound_names is not None:
            for name in sound_names:
                self.add_sound(name)

    def add_sound(self, sound):
        """
        Adds a sound if it was not already added.

        :type sound: str
        """
        if sound in self._sounds:
            self._node.get_logger().warn(f'Sound {sound} is already present in sounds.')
        # else:
            # self._sounds[sound] = self._sound_client.waveSound(os.path.join(self._sounds_dir, '{0}.wav'.format(sound)))

    def play(self, sound):
        """
        Plays the given sound if it was loaded.

        :type sound: str
        :param sound: Name of the sound to play
        """
        if sound in self._sounds:
            self._node.get_logger().debug('Playing sound {0}'.format(sound))
            # self._sounds[sound].play()
        else:
            self._node.get_logger().warn('Sound {0} is not available'.format(sound))
