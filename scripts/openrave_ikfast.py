import time
import openravepy
from openravepy import *
from openravepy import ikfast

from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()

env = Environment()
# module = RaveCreateModule(env, 'urdf')
# 
# name = module.SendCommand('load /home/dvrk/ecm.urdf')
# kinbody = env.GetKinBody(name)
 
kinbody = env.ReadRobotXMLFile('/home/dvrk/ecm.xml')
env.Add(kinbody)
  
print ('degrees of freedom = {}\n\n\n'.format(kinbody.GetDOF()) )
solver = ikfast.IKFastSolver(kinbody=kinbody)
chaintree = solver.generateIkSolver(baselink=0,eelink=7,freeindices=[],solvefn=ikfast.IKFastSolver.solveFullIK_TranslationAxisAngle4D)
code = solver.writeIkSolver(chaintree)
open('ik.cpp','w').write(code)
 
print(code)