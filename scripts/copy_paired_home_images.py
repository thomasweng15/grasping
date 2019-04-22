
# Sort and output to snapshots-sorted
# with open("snapshots.txt", "r") as f:
#     fnames = sorted(f)

# with open('snapshots-sorted.txt', 'w') as f:
#     for item in fnames:
#         if '~' in item:
#             continue
#         f.write("%s" % item)

num_files = sum(1 for line in open('snapshots-sorted.txt'))
num_trials = num_files / 2
trial_duration = 30 # seconds per trial
period = 3 * 60 * 60 # num hrs in seconds
num_skip = period / trial_duration
print("How many trials to skip", num_skip)
num_remaining = num_trials / num_skip
print("How many trials remain", num_remaining)

files = []
with open("snapshots-sorted.txt", "r") as f:
    for line in f.readlines():
        files.append(line)

print("Total number of files", len(files))

snapshots_files = []
for i in range(num_remaining):
    idx = i*(num_skip + 2)
    fname_depth = files[idx]
    fname_rgb = files[idx + 1]
    snapshots_files.append(fname_depth)
    snapshots_files.append(fname_rgb)

print("Remaining files", len(snapshots_files))

with open('snapshots-remaining-3h.txt', 'w') as f:
    for item in snapshots_files:
        f.write("%s" % item)