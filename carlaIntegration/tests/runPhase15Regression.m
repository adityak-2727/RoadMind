function summary = runPhase15Regression()
% Run original assertions unchanged. Never relabel skipped tests as passed.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(root));
outDir = fullfile(root,'results','phase15',['regression_' char(datetime('now','Format','yyyyMMdd_HHmmss'))]);
mkdir(outDir); diary(fullfile(outDir,'console.txt')); diaryGuard = onCleanup(@() diary('off')); %#ok<NASGU>
cfg = carlaConfig(); env = pyenv;
if env.Status == "NotLoaded"; pyenv('Version',cfg.pythonExecutable); end
files = {'carlaIntegration/tests/testPhase15DemoSafety.m','tests/testPrediction.m','tests/testPlanner.m','tests/testCollisionCheck.m', ...
    'carlaIntegration/tests/testCarlaIntegration.m','carlaIntegration/tests/testCarlaSensors.m', ...
    'carlaIntegration/tests/testCarlaPerceptionFusion.m','carlaIntegration/tests/testCarlaIndianHeroScene.m', ...
    'carlaIntegration/tests/testCarlaHeroSceneSensorFusion.m','carlaIntegration/tests/testCarlaHeroSceneTrackingPrediction.m', ...
    'carlaIntegration/tests/testCarlaPlanningDecision.m','carlaIntegration/tests/testCarlaPhase14Turning.m'};
summary = struct('suite',{},'passed',{},'failed',{},'incomplete',{},'exception',{});
for i = 1:numel(files)
    fprintf('\nPHASE15_REGRESSION %s\n',files{i});
    row = struct('suite',string(files{i}),'passed',0,'failed',0,'incomplete',0,'exception',"");
    try
        results = runtests(fullfile(root,files{i}));
        row.passed = sum([results.Passed]); row.failed = sum([results.Failed]); row.incomplete = sum([results.Incomplete]);
        save(fullfile(outDir,sprintf('suite_%02d.mat',i)),'results');
        writetable(table(string({results.Name})',[results.Passed]',[results.Failed]',[results.Incomplete]', ...
            'VariableNames',{'Name','Passed','Failed','Incomplete'}),fullfile(outDir,sprintf('suite_%02d.csv',i)));
    catch err
        row.exception = string(getReport(err,'extended','hyperlinks','off'));
    end
    summary(end+1) = row; %#ok<AGROW>
    writetable(struct2table(summary),fullfile(outDir,'summary.csv'));
    carlaDisconnect();
end
% Isolate main's relative output paths from existing scenario evidence.
oldDir = pwd; dirGuard = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(outDir);
scenarios = ["villageRoad","urbanIntersection","highwayMerge","marketArea","cattleCrossing"];
scenarioResults = struct();
for name = scenarios
    try
        scenarioResults.(name) = main(name,false);
    catch err
        scenarioResults.(name) = struct('exception',string(getReport(err,'extended','hyperlinks','off')));
    end
    save(fullfile(outDir,'five_scenarios.mat'),'scenarioResults');
end
end
