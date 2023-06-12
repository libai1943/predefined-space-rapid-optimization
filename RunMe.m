clear all; clc; close all;
global params_
Initial();

for ii = 1 : 115
    params_.user.case_id = ii;
    InitializeParams();
    SearchViaHAs();
    OptiViaPSRO();
end