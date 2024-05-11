import jrl
import glob 
import os


def read_communication_counts(result_dir):
    result = []
    with open(os.path.join(result_dir, "communication_counts.txt")) as f:
        line = f.readline()
        for count in line.split(" "):
            if count:
                result.append(int(count))
    return result

def read_results_metrics_all(result_dir):
    parser = jrl.Parser()
    metric_summaries = []

    # Aggregate all the metric summary files
    metrics_files = sorted(glob.glob(os.path.join(result_dir, "iterations", "*.jrm*")))
    metrics_files.append(os.path.join(result_dir, "final_metrics.jrm.cbor"))
    
    # read the communication counts
    comm_counts = read_communication_counts(result_dir)

    for count, mf in zip(comm_counts, metrics_files):
        if os.path.isfile(mf):
            metric_summaries.append((count, parser.parseMetricSummary(mf, True)))
        else:
            return None
    return metric_summaries


