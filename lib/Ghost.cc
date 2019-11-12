#include "Ghost.h"

using namespace opencog;
using namespace std;

Ghost::Ghost(AtomSpace *_as, bool _one_to_one, int _resp_wait_sec) : g_as(_as)
{
    g_one_to_one = _one_to_one;
    g_resp_wait_sec = _resp_wait_sec;
    g_inited = false;
    g_one_to_one = false;

    g_cs = new CogServer(_as);
    g_se = new SchemeEval(_as);

}

Ghost::~Ghost()
{
}

void Ghost::setRelexServer(std::string _r_hostname, std::string _r_port)
{
    if(g_inited)
        g_se->eval(string("(use-relex-server \"") +
                        _r_hostname + "\" " + _r_port + ")");
}

void Ghost::setGhostRuleFiles(std::vector<std::string> _gr_files)
{
    g_gr_files = std::vector<std::string>(_gr_files);
}

void Ghost::setSchemeModules(std::vector<std::string> _gs_modules)
{
    g_gs_modules = std::vector<std::string>(_gs_modules);
}

void Ghost::loadRuleFile(std::string &output, const std::string &file_path)
{
    string out_msg = "";
    printf("LOADING GHOST -> %s\n", file_path.c_str());
    output = g_se->eval("(ghost-parse-file \"" + file_path + "\")");
    output.assign(out_msg);
}

void Ghost::loadSchemeModule(std::string &output, const std::string &file_path)
{
    string out_msg = "";
    output = g_se->eval("(load \"" + file_path + "\")");
    printf("LOADING SCM -> %s\n>>>>>>>>%s\n", file_path.c_str(), output.c_str());
    output.assign(out_msg);
}

void Ghost::scmEval(std::string &rOut, const std::string &scm_str)
{
    rOut = g_se->eval(scm_str);
}

void Ghost::ghostInit()
{
    // Init ghost
    // TODO handle errors
    ///     attention module
    g_cs_modules.push_back("/usr/local/lib/opencog/libattention.so");

    g_cs_agents.push_back("opencog::AFImportanceDiffusionAgent");
    g_cs_agents.push_back("opencog::WAImportanceDiffusionAgent");
    g_cs_agents.push_back("opencog::AFRentCollectionAgent");
    g_cs_agents.push_back("opencog::WARentCollectionAgent");

    g_inited = true;
    setRelexServer("localhost", "4444");

    std::string output;
    // load guile scheme modules
    for(int i = 0; i < g_gs_modules.size(); i++)
        loadSchemeModule(output, g_gs_modules[i]);
    // load ghost rules
    for(int i = 0; i < g_gr_files.size(); i++)
        loadRuleFile(output, g_gr_files[i]);
    printf("** Done Initialization **\n>>%s<<\n", output.c_str());
}

void Ghost::ghostRun()
{
    g_se->eval("(ghost-run)");
}

void Ghost::eraseSubStr(std::string & mainStr, const std::string & toErase)
{
    size_t pos = mainStr.find(toErase);
    if (pos != std::string::npos)
        mainStr.erase(pos, toErase.length());
}

void Ghost::getGhostResponse(std::string &rOutput, int wait_for_response_secs)
{
    string output = "";
    // TODO replace the wait with something more dynamic.
    //      ghost should let it know when it has a response
    usleep(wait_for_response_secs * 1000000);
    output = g_se->eval("(ghost-str-response)");
    if (output.back() == '\n') output.pop_back();
    std::string rm = "finished-action";
    eraseSubStr(output, rm);
    if (output.length() > 0) {
        rOutput.assign(output);
        return;
    }
    rOutput.assign("");
}

void Ghost::utterance(const string &rUtterance, string &rOutput)
{
    rOutput = g_se->eval("(ghost \"" + rUtterance + "\")");
    if(this->g_one_to_one)
        getGhostResponse(rOutput, g_resp_wait_sec);
}

void Ghost::ghostShutdown()
{
    g_se->eval("(ghost-halt)");
    usleep(2000);
}
