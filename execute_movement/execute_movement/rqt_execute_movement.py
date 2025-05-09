from rqt_py_common.plugin_container_widget import PluginContainerWidget
from rqt_gui_py.plugin import Plugin

from execute_movement.rqt_execute_movement_widget import RqtExecute_MovementWidget


class RqtExecute_Movement(Plugin):
    def __init__(self, plugin_context):
        super(RqtExecute_Movement, self).__init__(plugin_context)
        self._plugin_context = plugin_context

        self.mainwidget = RqtExecute_MovementWidget(self, plugin_context)

        plugin_context.add_widget(self.mainwidget)

    def get_widget(self):
        return self.mainwidget

    def shutdown_plugin(self):
        self.mainwidget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        self.mainwidget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self.mainwidget.restore_settings(plugin_settings, instance_settings)

