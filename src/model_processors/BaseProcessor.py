"""
Object factory for object-detection tasks - returns a fully initialized object 
"""
import os 

from atlas_utils.acl_resource import AclResource
from atlas_utils.acl_model import Model

class BaseProcessor:
    def __init__(self, params):
        # Initialize ACL Resources
        self._acl_resource = AclResource()
        self._acl_resource.init()
        self.params = params
        self._model_width = params['model_width']
        self._model_height = params['model_height']
        assert 'model_path' in params and params['model_path'] is not None, 'Review your param: model_path'
        assert os.path.exists(params['model_path']), "Model directory doesn't exist {}".format(params['model_path'])
        # Init Model
        self.model = Model(params['model_path'])

    def preprocess(self):
        raise NotImplementedError
    
    def postprocess(self):
        raise NotImplementedError