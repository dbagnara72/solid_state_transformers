# Solid state transformers

This repo contains a collection of simscape models concerning the Solid State Transformers (SSTs). Basically the repo is a on working study on different SSTs architectures. 

**How to use this repository**:
- Add to matlab **path/with-subfolders** the repository **library**. The library repo contains all fundamentals simscape (ssc) models compiled with "ssc_bultd".
    - The folder ./library/user_defined_functions/ccaller contains a list of c-coded functions used inside the simulink models by ccallers.
    - The folder ./library/foundation contains all simscape language based model used in the simulink models.
    - The folder ./library/documentations contains documentation for most of the applications available on **modelization-and-control**, **solid_state_transformers**, and others repositories.

 # Documentation
Documentation is available into the **library** repository.

 # Description of the repo

As known SST are build by a cascade of single-phase-inverter, where each one is galvanically insulated by DABs, LLCs and similar high efficient DC/DC converters.

The repo investigates on finding an optimal implementation in terms of efficiency and controllability.

Here a description of what folders contains.

**Remark** - each models implement a local time management on each DC/DC or DC/AC as well. Modulators generates trigger for the control system and the models permit 
to implement effects on local time sliding.

**theory analysis solid state transformer**:
Investigantion on different SST architectures:

For an easier description of the proposed architecture the term *module* is clarified as will be used.
**module**: *module* means a block composed by an galvanically insolated DC/DC followed by a single phase inverter.

***Description of the folders.***
The taxonomy concerning the proposed architectures starts classifying the single phase inverter architectures following on different isolated DC/DC architectures, as follows.

- two level full bridge;
- three level NPC full bridge;
- three level T-Type full bridge;

- single phase DAB;
- single phase resonant CLLC;
- three phase DAB.

In particular:

**folder**: *sst_based_on_full_bridge_npc_inverter\sst_single_phase_dab_single_phase_npc_inv*: contains a two modules series/parallel based on single phase 
DAB and a single phase inverted based on a three level NPC full bridge (H-bridge).

**folder**: *sst_based_on_full_bridge_npc_inverter\sst_single_phase_cllc_single_phase_npc_inv*: contains a two modules series/parallel based on single phase 
resonant CLLC and a single phase inverted based on a three level NPC full bridge (H-bridge).

**folder**: *sst_based_on_full_bridge_npc_inverter\sst_three_phase_dab_single_phase_npc_inv*: contains a two modules series/parallel based on three phase 
DAB and a single phase inverted based on a three level NPC full bridge (H-bridge).

**folder**: *sst_based_on_full_bridge_ttype_inverter\sst_three_phase_dab_single_phase_ttype_inv*: contains a two modules series/parallel based on three phase 
DAB and a single phase inverted based on a three level t-type full bridge (H-bridge).


**Remark**: CLLC is tuned to achieve ZCS, and power flow is controlled by phase shift between primary/secondary full-bridges.

**Some implemented details**
- three phase DAB modulator run at 24kHz where at every step move one adjacent space vector resulting in a fundamental frequency of 4kHz;  
- single phase CLLC runs at constant frequency of 13kHz, and power flow is controlled by phase shifting between primary and secondary;  
- single phase DAB runs at constant frequency of 12kHz, and power flow is controlled by phase shifting between primary and secondary;  

Results are presented in the document **library\documentation\solid_state_transformers\solid_state_transformers**

- model implements two dab connected in parallel at battery side (800V);
- output voltage of the iso DC/DC is 800V;
- each dab supplies a single phase inverter (270Vac);
- single phase inverters are connected in series (540Vac per phase);
- hw and sw implementation;
- n-independent time domains.