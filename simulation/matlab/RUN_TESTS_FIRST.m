% RUN_TESTS_FIRST  Quick entry point for validation tests only.
%
% Use this before full plotting if you want a quick sanity pass.

clearvars;
cases = run_validation_cases(); %#ok<NASGU>
