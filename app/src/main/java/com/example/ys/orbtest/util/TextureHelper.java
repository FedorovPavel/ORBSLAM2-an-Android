package com.example.ys.orbtest.util;

/**
 * Created by dell on 2017/12/28.
 */

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.opengl.GLES20;
import android.opengl.GLUtils;
import android.util.Log;

public class TextureHelper {

    public static final String TAG = "TextureHelper";

    public static int loadTexture(Context context, int resourceId, boolean isRepeat){

        final int[] textureObjectId = new int[1];
        GLES20.glGenTextures(1,textureObjectId, 0);
        if(textureObjectId[0] == 0){
            if(LoggerConfig.ON){
                Log.w(TAG,"Could not generate a new Opengl texture object");
            }
            return 0;
        }

        final BitmapFactory.Options options = new BitmapFactory.Options();
        options.inScaled = false;
        final Bitmap bitmap = BitmapFactory.decodeResource(context.getResources(),resourceId, options);
        if(bitmap == null){
            if(LoggerConfig.ON){
                Log.w(TAG,"ResourceId:"+resourceId+"could not be decoded");
            }
            GLES20.glDeleteTextures(1, textureObjectId, 0);
            return 0;
        }
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D,textureObjectId[0]);

        if(isRepeat){
            GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_REPEAT);
            GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_REPEAT);
        }


        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR_MIPMAP_LINEAR);

        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);

        GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bitmap, 0);
        bitmap.recycle();
        GLES20.glGenerateMipmap(GLES20.GL_TEXTURE_2D);
        return textureObjectId[0];
    }
}