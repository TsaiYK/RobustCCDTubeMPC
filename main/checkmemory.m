function checkmemory()
userview = memory;
if userview.MemUsedMATLAB >= userview.MemAvailableAllArrays*0.9
    error('Out of Memory; STOP!')
end
