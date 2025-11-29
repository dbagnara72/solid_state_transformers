# Solid state transformers

This repo contains a collection of simscape models concerning different configuration of SSTs. 

**How to use the repo**:
- Add to matlab **path/with-subfolders** the repo **library**. This repo contains all fundamentals simscape (ssc) models compiled with "ssc_bultd".
    - The folder ./library/user_defined_functions/ccaller contains a list of c-coded functions used inside the simulink models by ccallers.
    - The folder ./library/foundation contains all simscape language based model used in the simulink models.
    - The folder ./library/documentations contains documentation for most of the applications available on **modelization-and-control** and **solid_state_transformers** repos.

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

**module**: *module* menas a block composed by an galvanically insolated DC/DC followed by a single phase inverter.

**folder**: *sst_single_phase_dab_single_phase_inv*: contains a two modules series/parallel based on single phase DAB and single phase inverted based on a full bridge (H-bridge).
**folder**: *sst_single_phase_dab_single_phase_npc_inv*: contains a two modules series/parallel based on single phase DAB and single phase inverted based on a three level NPC full bridge (H-bridge).

**folder**: *sst_single_phase_LLC_single_phase_npc_inv*: contains a two modules series/parallel based on single phase LLC and single phase inverted based on a three level NPC full bridge (H-bridge).

**folder**: *sst_three_phase_dab_single_phase_inv*: contains a two modules series/parallel based on three phase DAB and single phase inverted based on a full bridge (H-bridge).
**folder**: *sst_three_phase_dab_single_phase_npc_inv*: contains a two modules series/parallel based on three phase DAB and single phase inverted based on a three level NPC full bridge (H-bridge).

**Remark**: LLC is kept at the frequency which ensure ZCS, power flow is controlled by phase shift between primary/secondary H-bridges.

- three phase DAB: very high efficiency on both semiconductor and magnetics, and high controllability, fundamental of the three phase at 4kHz;  
- single phase LLC: very high efficiency on semiconductor applying DCM that means ZCS, but controllability a little poor, magnetics run at 9.6kHz;  
- single phase DAB: very high efficiency on semiconductor wirth proper deat-time that enables ZVS on turn-on, very high controllability, magnetics run at 9.6kHz;

- model implements two dab connected in parallel at battery side (1.25kV);
- output voltage of the iso DC/DC is 1.5kV;
- each dab supplies a single phase inverter (400Vac);
- single phase inverters are connected in series (800Vac per phase);
- hw and sw implementation;
- n-independent time domains;

Improvements (maybe): LV DC grid at 800V, high voltage AC component at 3.3kV;
