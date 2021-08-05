from simple_term_menu import TerminalMenu

class ModuleSelector():
    def __init__(self, params):
        self.params = params
            
    def user_input(self):
        input_task = input("Choose task (C - Classification, O - Object Detection, T - Object Tracking, D - Depth Estimation, S - Segmentation): ")
        # select classification model - print the avaiable models to users and let them choose
        if input_task.lower() == "c":
            self.task  = "classification"
        elif input_task.lower() == "o":
            self.task = "object_detection"
        elif input_task.lower() == "d":
            self.task = "depth_estimation"
        elif input_task.lower() == "t":
            self.task = "object_tracking"
        self._select_model()

    def _select_model(self):
        """Internal method to select the model given a task. User would not need to call this normally."""
        available_models = self.params["task"][self.task]
        models_list = list(available_models.keys())
        models_list.append("Return to task selection")
        
        # prompt CLI models options
        terminal_menu = TerminalMenu(models_list, preview_command=self._preview_model, 
            preview_size=0.75)
        menu_entry_index = terminal_menu.show()

        self.model = models_list[menu_entry_index]
        self.processor = self.params["task"][self.task][self.model]["model_processor"]
        # go back and select another task, model, etc - prompt user-input
        if self.model == "Return to task selection":
            self.user_input()

    def _preview_model(self, model):
        """Internal method to preview the contents of a model during model selection - user would not call this normally"""
        if model == "Return to task selection":
            return
        model_info = self.params["task"][self.task][model]
        preview_output = f"""
        {model}:\nInput shape (WxH): {model_info["model_width"]} x {model_info["model_height"]}\nDefault offline model (.om): {model_info["model_path"]}\nInfo: Visit {model_info["model_info"]}\nModelProcessor: {model_info["model_processor"]}\n
        """
        return preview_output

