#ifndef GHOST_SERVICE_H
#define GHOST_SERVICE_H

#include <vector>
#include <string>
#include <ctime>                                         

#include <boost/algorithm/string/classification.hpp>     
#include <boost/range/algorithm_ext/erase.hpp>           
#include <boost/lexical_cast.hpp>

#define HAVE_GUILE

#include <opencog/cogserver/server/CogServer.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/guile/SchemeEval.h>

using namespace opencog;

class Ghost
{
private:
    AtomSpace *g_as;
    CogServer *g_cs;
    SchemeEval *g_se;

    std::string g_relex_hostname;
    std::string g_relex_port;

    std::vector<std::string> g_cs_modules; // cogserver modules
    std::vector<std::string> g_gs_modules; // guile scheme modules
    std::vector<std::string> g_cs_agents;  // cogserver mind agents
    std::vector<std::string> g_gr_files;  // ghost rule files

    bool g_one_to_one;
    bool g_inited;
    int g_resp_wait_sec;
    
public:
    Ghost(AtomSpace *_as,
          bool _one_to_one=false,
          int _resp_wait_sec=5) ;
    ~Ghost();
    
    void setCogServerModules(std::vector<std::string> _cs_modules);
    void setGuileModules(std::vector<std::string> _gs_modules);
    void setCogServerAgents(std::vector<std::string> _cs_agents);
    void setRelexServer(std::string _r_hostname, std::string _r_port);
    void setSchemeModules(std::vector<std::string> _gs_modules);
    void setGhostRuleFiles(std::vector<std::string> _gr_files);
    void eraseSubStr(std::string & mainStr, const std::string & toErase);
    void getGhostResponse(std::string &rOutput, int attempts_time);
    void loadRuleFile(std::string &output, const std::string &file_path);
    void loadSchemeModule(std::string &output, const std::string &file_path);
    void scmEval(std::string &rOut, const std::string &scm_str);
    void ghostInit();
    void ghostRun();
    void ghostShutdown();
    void utterance(const std::string &rUtterance, std::string &rOutput);
};
#endif
