# Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
import sys
import os
import shutil
from setuptools.command.install import install as install_orig
from distutils.command.build import build as build_orig
from distutils.command.clean import clean as clean_orig
from setuptools import setup,find_packages
import glob

packageName="ASRCAISim1"
build_config="Debug" if "--Debug" in sys.argv else "Release"
include_addons=[
    "rayUtility",
    "AgentIsolation",
    "MatchMaker",
    "torch_truncnorm",
    "HandyRLUtility",
]
exclude_addons=[]
include_samples=[
    "modules/OriginalModelSample",
    "scripts/R5Contest/MinimumEvaluation",
    "scripts/R5Contest/HandyRLSample",
    "scripts/R5Contest/raySample",
]
i=0
while i<len(sys.argv):
    arg=sys.argv[i]
    if(arg.startswith("--addons")):
        include_addons.extend(arg[9:].split(","))
        sys.argv.pop(i)
    elif(arg.startswith("--ex-addons")):
        exclude_addons.extend(arg[12:].split(","))
        sys.argv.pop(i)
    elif(arg=="--Debug" or arg=="--Release"):
        sys.argv.pop(i)
    else:
        i+=1
exclude_addons=list(set(exclude_addons))
include_addons=list(set(include_addons))
i=0
while i<len(include_addons):
    if(include_addons[i] in exclude_addons):
        include_addons.pop(i)
    else:
        i+=1
isMSYS=False

class build(build_orig):
    def run(self):
        import sys
        import subprocess
        import numpy
        import pybind11
        prefix=sys.base_prefix
        py_ver="%d.%d" % sys.version_info[:2]
        os.environ["PythonLibsNew_FIND_VERSION"]=py_ver
        if(os.name=="nt"):
            python_include_dir=os.path.dirname(glob.glob(os.path.join(prefix,"include","Python.h"))[0])
            python_lib_dir=os.path.join(prefix,"libs")
        else:
            python_include_dir=os.path.dirname(glob.glob(os.path.join(prefix,"include/python"+py_ver+sys.abiflags+"/Python.h"))[0])
            python_lib_dir=os.path.join(prefix,"lib")
        numpy_include_dir=numpy.get_include()
        pybind11_cmake_dir=pybind11.get_cmake_dir()
        if(os.name=="nt"):
            if(isMSYS):
                subprocess.check_call([".\\builder_MSYS.bat",
                    build_config,
                    python_include_dir.replace(os.path.sep,'/'),
                    python_lib_dir.replace(os.path.sep,'/'),
                    numpy_include_dir.replace(os.path.sep,'/'),
                    pybind11_cmake_dir.replace(os.path.sep,'/')
                ])
            else:
                subprocess.check_call([".\\builder.bat",
                    build_config,
                    python_include_dir.replace(os.path.sep,'/'),
                    python_lib_dir.replace(os.path.sep,'/'),
                    numpy_include_dir.replace(os.path.sep,'/'),
                    pybind11_cmake_dir.replace(os.path.sep,'/')
                ])
            dummy=os.path.join(os.path.dirname(__file__),packageName,"libCore.pyd")
            if(os.path.exists(dummy)):
                os.remove(dummy)
            f=open(dummy,"w") #Dummy
            f.close()
        else:
            subprocess.check_call(["bash","./builder.sh",
                build_config,
                python_include_dir,
                python_lib_dir,
                numpy_include_dir,
                pybind11_cmake_dir
            ])
        core_simulator_dir=os.path.abspath(os.path.join(os.path.dirname(__file__),packageName))
        addon_base_dir=os.path.abspath(os.path.join(os.path.dirname(__file__),"addons"))
        cwd=os.getcwd()
        for addon in include_addons:
            addon_src_dir=os.path.join(addon_base_dir,addon)
            os.chdir(addon_src_dir)
            addon_dst_dir=os.path.join(core_simulator_dir,"addons",addon)
            addon_include_dir=os.path.join(core_simulator_dir,"include",packageName,"addons",addon)
            import importlib
            spec=importlib.util.spec_from_file_location("builder","builder.py")
            builder=importlib.util.module_from_spec(spec)
            spec.loader.exec_module(builder)
            builder.run(
                build_config,
                core_simulator_dir.replace(os.path.sep,'/'),
                addon_dst_dir.replace(os.path.sep,'/'),
                addon_include_dir.replace(os.path.sep,'/'),
                addon,
                isMSYS
            )
            del builder
            os.chdir(cwd)
            if(os.name=="nt"):
                if(os.path.exists(os.path.join(addon_dst_dir,"lib"+addon+".dll")) or
                    os.path.join(addon_dst_dir,addon+".dll")):
                    dummy=os.path.join(addon_dst_dir,"lib"+addon+".pyd")
                    if(os.path.exists(dummy)):
                        os.remove(dummy)
                    f=open(dummy,"w") #Dummy
                    f.close()
        build_orig.run(self)

class install(install_orig):
    user_options = install_orig.user_options + [
        ("MSYS",None,"whether use MSYS or not")
    ]
    
    def initialize_options(self):
        install_orig.initialize_options(self)
        self.MSYS = None
    def finalize_options(self):
        install_orig.finalize_options(self)
        global isMSYS
        isMSYS = os.name=="nt" and self.MSYS is not None
    def run(self):
        install_orig.run(self)
        if(os.name=="nt"):
            dummy=os.path.join(os.path.dirname(__file__),packageName+"/libCore.pyd")
            if(os.path.exists(dummy)):
                os.remove(dummy) #Dummy
            core_simulator_dir=os.path.join(os.path.dirname(__file__),packageName)
            for addon in include_addons:
                dummy=os.path.join(core_simulator_dir,"addons/"+addon,"lib"+addon+".pyd")
                if(os.path.exists(dummy)):
                    os.remove(dummy) #Dummy

class clean(clean_orig):
    def run(self):
        clean_orig.run(self)
        base_dir=os.path.dirname(__file__)
        build_dir=os.path.join(base_dir, "build")
        dist_dir=os.path.join(base_dir, "dist")
        core_simulator_dir=os.path.join(base_dir,packageName)
        module_include_dir=os.path.join(core_simulator_dir,"include")
        shutil.rmtree(build_dir,True)
        shutil.rmtree(dist_dir,True)
        os.remove(os.path.join(base_dir,"MANIFEST.in"))
        shutil.rmtree(os.path.join(base_dir,packageName+".egg-info"),True)
        addon_base_dir=os.path.abspath(os.path.join(os.path.dirname(__file__),"addons"))
        for addon in include_addons:
            addon_src_dir=os.path.join(addon_base_dir,addon)
            addon_dst_dir=os.path.join(core_simulator_dir,"addons",addon)
            addon_include_dir=os.path.join(module_include_dir,packageName,"addons",addon)
            import importlib
            spec=importlib.util.spec_from_file_location("builder",os.path.join(addon_src_dir,"builder.py"))
            builder=importlib.util.module_from_spec(spec)
            spec.loader.exec_module(builder)
            if hasattr(builder,"clean"):
                builder.clean(
                    base_dir.replace(os.path.sep,'/'),
                    core_simulator_dir.replace(os.path.sep,'/'),
                    addon_src_dir.replace(os.path.sep,'/'),
                    addon_dst_dir.replace(os.path.sep,'/'),
                    addon_include_dir.replace(os.path.sep,'/'),
                    addon
                )
            else:
                shutil.rmtree(os.path.join(addon_src_dir,"build"),True)
                shutil.rmtree(os.path.join(addon_src_dir,"__pycache__"),True)
                shutil.rmtree(addon_dst_dir,True)
                shutil.rmtree(addon_include_dir,True)
            del builder
        shutil.rmtree(module_include_dir,True)
        shutil.rmtree(os.path.join(core_simulator_dir,"share","cmake","addons"),True)
        for f in glob.glob(os.path.join(core_simulator_dir,"*Core.*")):
            os.remove(f)
        for d in glob.glob(os.path.join(base_dir,"**/__pycache__")):
            shutil.rmtree(d,True)

version=open("version.txt").read().strip()
requirements=open("requirements.txt","r").read().splitlines()
headers=glob.glob(packageName+"/include/"+packageName+"/**",recursive=True)+\
    glob.glob(packageName+"/include/thirdParty/**",recursive=True)
configs=glob.glob(packageName+"/config/*.json")
extra_requires={}
manifest=open("MANIFEST.in","w")
with open("MANIFEST.in.fragment","r") as fragment:
    manifest.write(fragment.read()+"\n")
for sample in include_samples:
    with open(os.path.join(os.path.dirname(__file__),"sample",sample,"MANIFEST.in.fragment"),"r") as fragment:
        manifest.write(fragment.read()+"\n")
core_simulator_dir=os.path.abspath(os.path.join(os.path.dirname(__file__),packageName))
addon_base_dir=os.path.abspath(os.path.join(os.path.dirname(__file__),"addons"))
for addon in include_addons:
    addon_src_dir=os.path.join(addon_base_dir,addon)
    addon_dst_dir=os.path.join(core_simulator_dir,"addons/"+addon)
    addon_include_dir=os.path.join(core_simulator_dir,"include/"+packageName+"/addons/"+addon)
    with open(os.path.join(addon_src_dir,"MANIFEST.in.fragment"),"r") as fragment:
        manifest.write(fragment.read()+"\n")
    if os.path.exists(os.path.join(addon_src_dir,"requirements.txt")):
        requirements.extend(open(os.path.join(addon_src_dir,"requirements.txt"),"r").read().splitlines())
manifest.close()

setup(
    name=packageName,
    version=version,
    author="Air Systems Research Center, ATLA",
    packages=find_packages(),
    include_package_data=True,
    cmdclass={"build":build,"install":install,"clean":clean},
    setup_requires=["numpy","pybind11>=2.10.2"],
    install_requires=requirements,
    license=("license.txt"),
    extras_require=extra_requires
)
