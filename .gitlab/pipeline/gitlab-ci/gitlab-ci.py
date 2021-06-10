import gcip

def create_pipeline():
    pipeline = gcip.Pipeline()
    pipeline.add_children(gcip.Job(stage="print_date", script="date"))
    return pipeline

def write_pipeline(pipeline):
    pipeline.write_yaml()

def main():
    pipeline = create_pipeline()
    write_pipeline(pipeline)

if __name__ == "__main__":
    main()
