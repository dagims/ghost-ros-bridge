#include "Ghost.h"

using namespace opencog;
using namespace std;

Ghost::Ghost(bool _one_to_one, int _resp_wait_sec)
{
    g_one_to_one = _one_to_one;
    g_resp_wait_sec = _resp_wait_sec;
    g_inited = false;
    g_one_to_one = false;
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

void Ghost::loadRuleFile(std::string &output, const std::string &file_path)
{
    string out_msg = "";
    output = g_se->eval("(ghost-parse-file \"" + file_path + "\")");
    output.assign(out_msg);
}

void Ghost::loadSchemeModule(std::string &output, const std::string &file_path)
{
    string out_msg = "";
    output = g_se->eval("(load \"" + file_path + "\")");
    output.assign(out_msg);
}

void Ghost::ghostInit()
{
    // Init ghost
    // TODO handle errors
    ///     attention module
    g_as = new AtomSpace();
    g_cs = new CogServer(g_as);
    g_se = new SchemeEval(g_as);
    
    g_cs_modules.push_back("/usr/local/lib/opencog/libattention.so");
    
    g_cs_agents.push_back("opencog::AFImportanceDiffusionAgent");
    g_cs_agents.push_back("opencog::WAImportanceDiffusionAgent");
    g_cs_agents.push_back("opencog::AFRentCollectionAgent");
    g_cs_agents.push_back("opencog::WARentCollectionAgent");

    g_se->eval("(use-modules (opencog)"
                          "(opencog exec)"
                          "(opencog openpsi)"
                          "(opencog nlp)"
                          "(opencog nlp relex2logic)"
                          "(opencog ghost)"
                          "(opencog ghost procedures)"
                          "(opencog logger))");
    
    g_se->eval("(ghost-set-sti-weight 0)");
    g_se->eval("(ghost-af-only #f)");
    g_se->eval("(define-public (append-space txt) (string-append txt " ")) ");
    g_inited = true;
    setRelexServer("localhost", "4444");
}

void Ghost::ghostRun()
{
    g_se->eval("(ghost-run)");
}

void Ghost::getGhostResponse(std::string &rOutput, int wait_for_response_secs)
{
    string output = "";

    usleep(wait_for_response_secs * 1000000);
    output = g_se->eval("(ghost-str-response)");
    if (output.back() == '\n') output.pop_back();
    if (output.length() > 0) {
        rOutput.assign(output);
        return;
    }
    rOutput.assign("I have nothing to say...");
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

