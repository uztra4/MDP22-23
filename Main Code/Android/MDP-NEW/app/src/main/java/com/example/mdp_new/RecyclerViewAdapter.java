package com.example.mdp_new;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import java.util.ArrayList;

public class RecyclerViewAdapter extends RecyclerView.Adapter<RecyclerViewAdapter.snapshotViewHolder>{

    ArrayList<String> data1;
    ArrayList<Integer> images;
    Context context;

    public RecyclerViewAdapter(Context ct, ArrayList<String> s1, ArrayList<Integer> img){
        context = ct;
        data1 = s1;
        images = img;
    }

    @NonNull
    @Override
    public snapshotViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        LayoutInflater inflater = LayoutInflater.from(context);
        View view = inflater.inflate(R.layout.my_row, parent, false);
        return new snapshotViewHolder(view);
    }

    @Override
    public void onBindViewHolder(@NonNull snapshotViewHolder holder, int position) {
        holder.myText1.setText(data1.get(position));
        holder.myImage.setImageResource(images.get(position));
    }

    @Override
    public int getItemCount() {
        return 0;
    }

    public class snapshotViewHolder extends RecyclerView.ViewHolder{

        TextView myText1;
        ImageView myImage;

        public snapshotViewHolder(@NonNull View itemView) {
            super(itemView);
            myText1 = itemView.findViewById(R.id.imageTitle);
            myImage = itemView.findViewById(R.id.imagePic);
        }
    }
}
