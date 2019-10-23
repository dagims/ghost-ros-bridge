#include "Ghost.h"

using namespace opencog;
using namespace std;

Ghost::Ghost()
{
}

Ghost::~Ghost()
{
}

void Ghost::loadRuleFile(std::string &output, const std::string &file_path)
{
    // status to be returned
    string out_msg = "";

    // load url into file
    string atomese_file_name = "";
    string error_msg = "";

    output = g_se->eval("(ghost-parse-file \"" + file_path + "\")");

    output.assign(out_msg);
}

void Ghost::ghost_init()
{
    // Init ghost
    // TODO handle errors
    ///     attention module
    g_as = new AtomSpace();
    g_cs = new CogServer(g_as);
    g_se = new SchemeEval(g_as);
    
    g_modules.push_back("/usr/local/lib/opencog/libattention.so");
    
    g_agents.push_back("opencog::AFImportanceDiffusionAgent");
    g_agents.push_back("opencog::WAImportanceDiffusionAgent");
    g_agents.push_back("opencog::AFRentCollectionAgent");
    g_agents.push_back("opencog::WARentCollectionAgent");

    g_se->eval("(use-modules (opencog)"
                          "(opencog exec)"
                          "(opencog openpsi)"
                          "(opencog nlp)"
                          "(opencog nlp relex2logic)"
                          "(opencog ghost)"
                          "(opencog ghost procedures)"
                          "(opencog logger))");
    g_relex_container_name = "localhost";
    g_se->eval(
        string("(use-relex-server \"") +
        g_relex_container_name +
        string("\" 4444)"));

    g_se->eval("(ghost-set-sti-weight 0)");
    g_se->eval("(ghost-af-only #f)");
    
    g_se->eval("(ghost-run)");
}

void Ghost::getGhostResponse(std::string &rOutput, double wait_for_response_secs)
{
    string output = "";

    usleep(wait_for_response_secs * 1000000);
    output = g_se->eval("(ghost-get-result)");
    printf("MAP COGNAME GHOST RESULT OUTPUT: %s\n", output.c_str());
    //boost::remove_erase_if(output, boost::is_any_of("\n()"));
    
    if (output.length() > 0) {
        rOutput.assign(output);
        return;
    }
    
    printf("GHOST RESPONSE CALL COMPLETED \n");
    rOutput.assign("I have nothing to say...");
}

void Ghost::utterance(const string &rUtterance, string &rOutput)
{
    rOutput = g_se->eval("(ghost \"" + rUtterance + "\")");
}

void Ghost::ghost_shutdown()
{
    g_se->eval("(ghost-halt)");
    usleep(2000);
}

