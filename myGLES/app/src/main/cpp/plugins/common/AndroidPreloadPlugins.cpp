#include "AndroidPreloadPlugins.h"
#include <cvrKernel/InteractionManager.h>
#include <cvrKernel/ComController.h>
#include <cvrConfig/ConfigManager.h>

#include <iostream>
#include <algorithm>
#include <sys/stat.h>


using namespace cvr;

PluginManager_An * PluginManager_An::_myPtr = NULL;

PluginManager_An::PluginManager_An()
{
}

PluginManager_An::~PluginManager_An()
{
    for(int i = 0; i < _loadedPluginList.size(); i++)
    {
        delete _loadedPluginList[i]->ptr;
    }
}

PluginManager_An * PluginManager_An::instance()
{
    if(!_myPtr)
    {
        _myPtr = new PluginManager_An();
    }
    return _myPtr;
}

bool PluginManager_An::init()
{
    std::vector<std::string> plugins;
    ConfigManager::getChildren("Plugin", plugins);

    for(int i = 0; i < plugins.size(); i++)
    {
        if(_pluginMap.find(plugins[i]) != _pluginMap.end())
        {
            continue;
        }
        _pluginMap[plugins[i]] = ConfigManager::getBool(
                std::string("Plugin") + "." + plugins[i],false);
    }

    // default nav buttons
    _pluginMap["MenuBasics"] = ConfigManager::getBool("Plugin.MenuBasics",true);

    for(std::map<std::string, bool>::iterator it = _pluginMap.begin();
            it != _pluginMap.end(); it++)
    {
        //std::cerr << it->first << " " << it->second << std::endl;
        if(it->second)
        {
            CVRPlugin * pluginPtr = ClassFactory::getInstance(it->first);
            if(!pluginPtr)
                it->second = false;
            else{
                it->second = true;
                PluginInfo * pi = new PluginInfo;
                pi->priority = pluginPtr->getPriority();
                pi->name = it->first;
                pi->ptr = pluginPtr;
                pi->path = "";
                _loadedPluginList.push_back(pi);
            }
        }
    }

    std::sort(_loadedPluginList.begin(),_loadedPluginList.end(),PrioritySort());

    std::cerr << "Loaded Plugins: " << std::endl;
    for(int i = 0; i < _loadedPluginList.size(); i++)
    {
        std::cerr << _loadedPluginList[i]->name << " Priority: "
                << _loadedPluginList[i]->priority << std::endl;
    }

    for(std::vector<PluginInfo *>::iterator it = _loadedPluginList.begin();
            it != _loadedPluginList.end();)
    {
        if(!(*it)->ptr->init())
        {
            _pluginMap[(*it)->name] = false;
            delete (*it);
            it = _loadedPluginList.erase(it);
            continue;
        }
        it++;
    }

    return true;
}

void PluginManager_An::preFrame()
{
    if(ComController::instance()->getIsSyncError())
    {
        return;
    }

    double startTime, endTime;

    osg::Stats * stats;
    stats = CVRViewer::instance()->getViewerStats();
    if(stats && !stats->collectStats("CalVRStats"))
    {
        stats = NULL;
    }

    if(stats)
    {
        startTime = osg::Timer::instance()->delta_s(
                CVRViewer::instance()->getStartTick(),
                osg::Timer::instance()->tick());
    }

    osg::Stats * statsPlugins;
    statsPlugins = CVRViewer::instance()->getViewerStats();
    if(statsPlugins && !statsPlugins->collectStats("CalVRStatsPlugins"))
    {
        statsPlugins = NULL;
    }

    for(int i = 0; i < _loadedPluginList.size(); i++)
    {
        double pluginsStartTime, pluginsEndTime;
        if(statsPlugins)
        {
            pluginsStartTime = osg::Timer::instance()->delta_s(
                    CVRViewer::instance()->getStartTick(),
                    osg::Timer::instance()->tick());
        }

        _loadedPluginList[i]->ptr->preFrame();

        if(statsPlugins)
        {
            pluginsEndTime = osg::Timer::instance()->delta_s(
                    CVRViewer::instance()->getStartTick(),
                    osg::Timer::instance()->tick());
            statsPlugins->setAttribute(
                    CVRViewer::instance()->getViewerFrameStamp()->getFrameNumber(),
                    _loadedPluginList[i]->name + " preFrame begin time",
                    pluginsStartTime);
            statsPlugins->setAttribute(
                    CVRViewer::instance()->getViewerFrameStamp()->getFrameNumber(),
                    _loadedPluginList[i]->name + " preFrame end time",
                    pluginsEndTime);
            statsPlugins->setAttribute(
                    CVRViewer::instance()->getViewerFrameStamp()->getFrameNumber(),
                    _loadedPluginList[i]->name + " preFrame time taken",
                    pluginsEndTime - pluginsStartTime);
        }
    }

    if(stats)
    {
        endTime = osg::Timer::instance()->delta_s(
                CVRViewer::instance()->getStartTick(),
                osg::Timer::instance()->tick());
        stats->setAttribute(
                CVRViewer::instance()->getViewerFrameStamp()->getFrameNumber(),
                "PreFrame begin time",startTime);
        stats->setAttribute(
                CVRViewer::instance()->getViewerFrameStamp()->getFrameNumber(),
                "PreFrame end time",endTime);
        stats->setAttribute(
                CVRViewer::instance()->getViewerFrameStamp()->getFrameNumber(),
                "PreFrame time taken",endTime - startTime);
    }
}

void PluginManager_An::postFrame()
{
    if(ComController::instance()->getIsSyncError())
    {
        return;
    }

    double startTime, endTime;

    osg::Stats * stats;
    stats = CVRViewer::instance()->getViewerStats();
    if(stats && !stats->collectStats("CalVRStatsAdvanced"))
    {
        stats = NULL;
    }

    if(stats)
    {
        startTime = osg::Timer::instance()->delta_s(
                CVRViewer::instance()->getStartTick(),
                osg::Timer::instance()->tick());
    }

    for(int i = 0; i < _loadedPluginList.size(); i++)
    {
        _loadedPluginList[i]->ptr->postFrame();
    }

    if(stats)
    {
        endTime = osg::Timer::instance()->delta_s(
                CVRViewer::instance()->getStartTick(),
                osg::Timer::instance()->tick());
        stats->setAttribute(
                CVRViewer::instance()->getViewerFrameStamp()->getFrameNumber(),
                "PostFrame begin time",startTime);
        stats->setAttribute(
                CVRViewer::instance()->getViewerFrameStamp()->getFrameNumber(),
                "PostFrame end time",endTime);
        stats->setAttribute(
                CVRViewer::instance()->getViewerFrameStamp()->getFrameNumber(),
                "PostFrame time taken",endTime - startTime);
    }
}

bool PluginManager_An::processEvent(InteractionEvent * event)
{
    for(int i = 0; i < _loadedPluginList.size(); i++)
    {
        if(_loadedPluginList[i]->ptr->processEvent(event))
        {
            return true;
        }
    }
    return false;
}

void PluginManager_An::sendMessageByName(std::string plugin, int type, char * data)
{
    for(int i = 0; i < _loadedPluginList.size(); i++)
    {
        if(_loadedPluginList[i]->name == plugin)
        {
            _loadedPluginList[i]->ptr->message(type,data,false);
            break;
        }
    }
}

bool PluginManager_An::getPluginLoaded(std::string plugin)
{
    if(_pluginMap.find(plugin) == _pluginMap.end())
    {
        return false;
    }

    return _pluginMap[plugin];
}

std::string PluginManager_An::getPluginName(CVRPlugin * plugin)
{
    for(int i = 0; i < _loadedPluginList.size(); ++i)
    {
        if(_loadedPluginList[i]->ptr == plugin)
        {
            return _loadedPluginList[i]->name;
        }
    }
    return "";
}

std::vector<std::string> PluginManager_An::getLoadedPluginList()
{
    std::vector<std::string> pluginList;

    for(int i = 0; i < _loadedPluginList.size(); i++)
    {
        pluginList.push_back(_loadedPluginList[i]->name);
    }

    return pluginList;
}

CVRPlugin * PluginManager_An::getPlugin(std::string plugin)
{
    CVRPlugin * ptr = NULL;
    for(int i = 0; i < _loadedPluginList.size(); i++)
    {
        if(_loadedPluginList[i]->name == plugin)
        {
            ptr = _loadedPluginList[i]->ptr;
            break;
        }
    }
    return ptr;
}

bool PluginManager_An::loadPlugin(std::string plugin)
{
    CVRPlugin * pluginPtr;
    CVRPlugin * (*func)();

    std::string libPath;

#ifdef WIN32
    libPath = plugin + ".dll";
#elif __APPLE__
    libPath = std::string("lib") + plugin + ".dylib";
#else
    libPath = std::string("lib") + plugin + ".so";
#endif

    for(int i = 0; i < _pluginLibDirs.size(); ++i)
    {
        struct stat sb;
        std::string testPath = _pluginLibDirs[i] + "/" + libPath;
        if(stat(testPath.c_str(),&sb) != -1)
        {
            libPath = testPath;
            break;
        }
    }

#ifndef WIN32
    char * error;
    void * libHandle;

    libHandle = dlopen(libPath.c_str(),RTLD_LAZY);
    if(!libHandle)
    {
        std::cerr << dlerror() << std::endl;
        return false;
    }

    func = (CVRPlugin * (*)())dlsym(libHandle, "createPlugin");if
(    (error = dlerror()) != NULL)
    {
        std::cerr << error << std::endl;
        return false;
    }
#else
    HINSTANCE libHandle;
    libHandle = LoadLibrary(libPath.c_str());
    if(!libHandle)
    {
        std::cerr << "Error: Unable to open DLL: " << libPath << std::endl;
        return false;
    }

    func = (CVRPlugin * (*)()) GetProcAddress(libHandle, "createPlugin");
    if(!func)
    {
        std::cerr << "Error: Unable to find function address in " << libPath << std::endl;
        return false;
    }
#endif

    pluginPtr = (*func)();
    int priority = pluginPtr->getPriority();

    PluginInfo * pi = new PluginInfo;
    pi->priority = priority;
    pi->name = plugin;
    pi->ptr = pluginPtr;
    pi->path = libPath;

    _loadedPluginList.push_back(pi);

    return true;
}