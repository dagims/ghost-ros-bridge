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

    std::string g_relex_container_name;

    std::vector<std::string> g_modules;
    std::vector<std::string> g_agents;
    
    int execute(std::string &rOutput, const std::vector<std::string> &rArgs);

public:
    Ghost();
    ~Ghost();

    void getGhostResponse(std::string &rOutput, double attempts_time);
    int getCommand(const std::string &rCmdStr);
    void loadRuleFile(std::string &output, const std::string &file_path);
    void ghost_init();
    void ghost_shutdown();
    void utterance(const std::string &rUtterance, std::string &rOutput);
};
#endif
