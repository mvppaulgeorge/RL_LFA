// Benchmark "adder" written by ABC on Wed Jul 17 15:43:42 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n339, new_n342, new_n344, new_n345, new_n346, new_n348, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d24x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand22aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n05x5   g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n06x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fiaoi012aa1n03p5x5 g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand22aa1n09x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand22aa1n12x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n112), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n114));
  nor002aa1n16x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nand02aa1n04x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanb02aa1n06x5               g021(.a(new_n115), .b(new_n116), .out0(new_n117));
  tech160nm_fixnrc02aa1n05x5   g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  nor043aa1d12x5               g023(.a(new_n114), .b(new_n117), .c(new_n118), .o1(new_n119));
  aoi112aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  nano23aa1n03x7               g025(.a(new_n113), .b(new_n110), .c(new_n111), .d(new_n112), .out0(new_n121));
  inv020aa1n04x5               g026(.a(new_n116), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\a[5] ), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\b[4] ), .o1(new_n124));
  tech160nm_fiaoi012aa1n05x5   g029(.a(new_n115), .b(new_n123), .c(new_n124), .o1(new_n125));
  nona22aa1n03x5               g030(.a(new_n121), .b(new_n122), .c(new_n125), .out0(new_n126));
  nona22aa1n03x5               g031(.a(new_n126), .b(new_n120), .c(new_n110), .out0(new_n127));
  xorc02aa1n12x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n127), .c(new_n109), .d(new_n119), .o1(new_n129));
  nor042aa1d18x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand42aa1n16x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  norb02aa1n06x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n129), .c(new_n98), .out0(\s[10] ));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n06x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  inv000aa1d42x5               g041(.a(new_n130), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n131), .o1(new_n138));
  aoai13aa1n04x5               g043(.a(new_n137), .b(new_n138), .c(new_n129), .d(new_n98), .o1(new_n139));
  nanp02aa1n09x5               g044(.a(new_n109), .b(new_n119), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n110), .o1(new_n141));
  inv000aa1n02x5               g046(.a(new_n120), .o1(new_n142));
  nor043aa1n02x5               g047(.a(new_n114), .b(new_n122), .c(new_n125), .o1(new_n143));
  nano22aa1n03x7               g048(.a(new_n143), .b(new_n141), .c(new_n142), .out0(new_n144));
  nand42aa1n04x5               g049(.a(new_n144), .b(new_n140), .o1(new_n145));
  norp02aa1n24x5               g050(.a(new_n130), .b(new_n97), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n131), .b(new_n147), .c(new_n145), .d(new_n128), .o1(new_n148));
  mtn022aa1n02x5               g053(.a(new_n139), .b(new_n148), .sa(new_n136), .o1(\s[11] ));
  nor042aa1n06x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nand42aa1n08x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  norb02aa1n06x4               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  aoi112aa1n02x5               g057(.a(new_n152), .b(new_n134), .c(new_n139), .d(new_n136), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n134), .o1(new_n154));
  nand42aa1n02x5               g059(.a(new_n139), .b(new_n136), .o1(new_n155));
  nanb02aa1n12x5               g060(.a(new_n150), .b(new_n151), .out0(new_n156));
  aoi012aa1n02x5               g061(.a(new_n156), .b(new_n155), .c(new_n154), .o1(new_n157));
  norp02aa1n02x5               g062(.a(new_n157), .b(new_n153), .o1(\s[12] ));
  nano32aa1n03x7               g063(.a(new_n156), .b(new_n128), .c(new_n132), .d(new_n136), .out0(new_n159));
  inv040aa1n03x5               g064(.a(new_n159), .o1(new_n160));
  nano32aa1n02x4               g065(.a(new_n156), .b(new_n154), .c(new_n135), .d(new_n131), .out0(new_n161));
  aoi012aa1n09x5               g066(.a(new_n150), .b(new_n134), .c(new_n151), .o1(new_n162));
  inv000aa1n02x5               g067(.a(new_n162), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n161), .c(new_n147), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n164), .b(new_n160), .c(new_n144), .d(new_n140), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  nand42aa1d28x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  aoi012aa1n03x5               g073(.a(new_n167), .b(new_n165), .c(new_n168), .o1(new_n169));
  xnrb03aa1n03x5               g074(.a(new_n169), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  aoai13aa1n02x5               g075(.a(new_n159), .b(new_n127), .c(new_n109), .d(new_n119), .o1(new_n171));
  nor002aa1n16x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand42aa1n10x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nona23aa1n02x4               g078(.a(new_n173), .b(new_n168), .c(new_n167), .d(new_n172), .out0(new_n174));
  oa0012aa1n12x5               g079(.a(new_n173), .b(new_n172), .c(new_n167), .o(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  aoai13aa1n02x7               g081(.a(new_n176), .b(new_n174), .c(new_n171), .d(new_n164), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nand42aa1n08x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  nor042aa1n04x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nanp02aa1n09x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nanb02aa1n02x5               g088(.a(new_n182), .b(new_n183), .out0(new_n184));
  inv000aa1d42x5               g089(.a(new_n184), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n179), .b(new_n185), .c(new_n177), .d(new_n181), .o1(new_n186));
  nano23aa1n06x5               g091(.a(new_n167), .b(new_n172), .c(new_n173), .d(new_n168), .out0(new_n187));
  aoai13aa1n03x5               g092(.a(new_n181), .b(new_n175), .c(new_n165), .d(new_n187), .o1(new_n188));
  oaoi13aa1n03x5               g093(.a(new_n184), .b(new_n188), .c(\a[15] ), .d(\b[14] ), .o1(new_n189));
  norp02aa1n03x5               g094(.a(new_n189), .b(new_n186), .o1(\s[16] ));
  nano23aa1n06x5               g095(.a(new_n134), .b(new_n150), .c(new_n151), .d(new_n135), .out0(new_n191));
  nano23aa1n06x5               g096(.a(new_n179), .b(new_n182), .c(new_n183), .d(new_n180), .out0(new_n192));
  nanp02aa1n03x5               g097(.a(new_n192), .b(new_n187), .o1(new_n193));
  nano32aa1n03x7               g098(.a(new_n193), .b(new_n191), .c(new_n132), .d(new_n128), .out0(new_n194));
  aoai13aa1n12x5               g099(.a(new_n194), .b(new_n127), .c(new_n109), .d(new_n119), .o1(new_n195));
  nona23aa1n09x5               g100(.a(new_n136), .b(new_n152), .c(new_n146), .d(new_n138), .out0(new_n196));
  aoi012aa1n06x5               g101(.a(new_n193), .b(new_n196), .c(new_n162), .o1(new_n197));
  nanp02aa1n03x5               g102(.a(new_n192), .b(new_n175), .o1(new_n198));
  aoi012aa1n02x5               g103(.a(new_n182), .b(new_n179), .c(new_n183), .o1(new_n199));
  nano22aa1n12x5               g104(.a(new_n197), .b(new_n198), .c(new_n199), .out0(new_n200));
  xorc02aa1n02x5               g105(.a(\a[17] ), .b(\b[16] ), .out0(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n195), .c(new_n200), .out0(\s[17] ));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(\b[16] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(new_n204), .b(new_n203), .o1(new_n205));
  nona23aa1n02x4               g110(.a(new_n183), .b(new_n180), .c(new_n179), .d(new_n182), .out0(new_n206));
  nor042aa1n03x5               g111(.a(new_n206), .b(new_n174), .o1(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n163), .c(new_n161), .d(new_n147), .o1(new_n208));
  nanp03aa1n02x5               g113(.a(new_n208), .b(new_n198), .c(new_n199), .o1(new_n209));
  aoai13aa1n02x5               g114(.a(new_n201), .b(new_n209), .c(new_n145), .d(new_n194), .o1(new_n210));
  nor002aa1d32x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  nanp02aa1n06x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  nanb02aa1n09x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  xobna2aa1n03x5               g118(.a(new_n213), .b(new_n210), .c(new_n205), .out0(\s[18] ));
  nanp02aa1n02x5               g119(.a(\b[16] ), .b(\a[17] ), .o1(new_n215));
  nano22aa1d15x5               g120(.a(new_n213), .b(new_n205), .c(new_n215), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoai13aa1n04x5               g122(.a(new_n212), .b(new_n211), .c(new_n203), .d(new_n204), .o1(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n217), .c(new_n195), .d(new_n200), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  nand02aa1n08x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  nanb02aa1d24x5               g128(.a(new_n222), .b(new_n223), .out0(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  inv040aa1d32x5               g130(.a(\a[20] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[19] ), .o1(new_n227));
  nand42aa1n04x5               g132(.a(new_n227), .b(new_n226), .o1(new_n228));
  nand02aa1n06x5               g133(.a(\b[19] ), .b(\a[20] ), .o1(new_n229));
  nand02aa1d08x5               g134(.a(new_n228), .b(new_n229), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  aoi112aa1n03x4               g136(.a(new_n222), .b(new_n231), .c(new_n219), .d(new_n225), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n222), .o1(new_n233));
  nanp02aa1n03x5               g138(.a(new_n219), .b(new_n225), .o1(new_n234));
  aoi012aa1n03x5               g139(.a(new_n230), .b(new_n234), .c(new_n233), .o1(new_n235));
  nor002aa1n02x5               g140(.a(new_n235), .b(new_n232), .o1(\s[20] ));
  nona22aa1n02x4               g141(.a(new_n216), .b(new_n224), .c(new_n230), .out0(new_n237));
  nand42aa1n03x5               g142(.a(new_n222), .b(new_n229), .o1(new_n238));
  nor043aa1n03x5               g143(.a(new_n218), .b(new_n224), .c(new_n230), .o1(new_n239));
  nano22aa1n03x7               g144(.a(new_n239), .b(new_n228), .c(new_n238), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n237), .c(new_n195), .d(new_n200), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[20] ), .b(\a[21] ), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  xnrc02aa1n12x5               g150(.a(\b[21] ), .b(\a[22] ), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  aoi112aa1n03x4               g152(.a(new_n243), .b(new_n247), .c(new_n241), .d(new_n245), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n243), .o1(new_n249));
  nanp02aa1n03x5               g154(.a(new_n241), .b(new_n245), .o1(new_n250));
  tech160nm_fiaoi012aa1n02p5x5 g155(.a(new_n246), .b(new_n250), .c(new_n249), .o1(new_n251));
  norp02aa1n03x5               g156(.a(new_n251), .b(new_n248), .o1(\s[22] ));
  nor042aa1n02x5               g157(.a(\b[19] ), .b(\a[20] ), .o1(new_n253));
  nona23aa1n09x5               g158(.a(new_n229), .b(new_n223), .c(new_n222), .d(new_n253), .out0(new_n254));
  nor042aa1n06x5               g159(.a(new_n246), .b(new_n244), .o1(new_n255));
  nanb03aa1n12x5               g160(.a(new_n254), .b(new_n255), .c(new_n216), .out0(new_n256));
  oai112aa1n06x5               g161(.a(new_n238), .b(new_n228), .c(new_n254), .d(new_n218), .o1(new_n257));
  tech160nm_fioaoi03aa1n03p5x5 g162(.a(\a[22] ), .b(\b[21] ), .c(new_n249), .o1(new_n258));
  aoi012aa1n02x5               g163(.a(new_n258), .b(new_n257), .c(new_n255), .o1(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n256), .c(new_n195), .d(new_n200), .o1(new_n260));
  xorb03aa1n02x5               g165(.a(new_n260), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  xorc02aa1n12x5               g167(.a(\a[23] ), .b(\b[22] ), .out0(new_n263));
  xorc02aa1n12x5               g168(.a(\a[24] ), .b(\b[23] ), .out0(new_n264));
  aoi112aa1n03x4               g169(.a(new_n262), .b(new_n264), .c(new_n260), .d(new_n263), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n262), .o1(new_n266));
  nand02aa1n02x5               g171(.a(new_n260), .b(new_n263), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n264), .o1(new_n268));
  tech160nm_fiaoi012aa1n03p5x5 g173(.a(new_n268), .b(new_n267), .c(new_n266), .o1(new_n269));
  nor042aa1n03x5               g174(.a(new_n269), .b(new_n265), .o1(\s[24] ));
  nand02aa1n02x5               g175(.a(new_n264), .b(new_n263), .o1(new_n271));
  nona23aa1n02x4               g176(.a(new_n255), .b(new_n216), .c(new_n271), .d(new_n254), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[22] ), .b(\a[23] ), .out0(new_n273));
  norb02aa1n06x5               g178(.a(new_n264), .b(new_n273), .out0(new_n274));
  norp02aa1n02x5               g179(.a(\b[23] ), .b(\a[24] ), .o1(new_n275));
  aoi112aa1n02x5               g180(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n276));
  nanp03aa1n06x5               g181(.a(new_n258), .b(new_n263), .c(new_n264), .o1(new_n277));
  nona22aa1n06x5               g182(.a(new_n277), .b(new_n276), .c(new_n275), .out0(new_n278));
  aoi013aa1n06x4               g183(.a(new_n278), .b(new_n257), .c(new_n255), .d(new_n274), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n272), .c(new_n195), .d(new_n200), .o1(new_n280));
  xorb03aa1n02x5               g185(.a(new_n280), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[25] ), .b(\b[24] ), .out0(new_n283));
  xorc02aa1n12x5               g188(.a(\a[26] ), .b(\b[25] ), .out0(new_n284));
  aoi112aa1n03x4               g189(.a(new_n282), .b(new_n284), .c(new_n280), .d(new_n283), .o1(new_n285));
  inv040aa1n03x5               g190(.a(new_n282), .o1(new_n286));
  nanp02aa1n03x5               g191(.a(new_n280), .b(new_n283), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n284), .o1(new_n288));
  tech160nm_fiaoi012aa1n02p5x5 g193(.a(new_n288), .b(new_n287), .c(new_n286), .o1(new_n289));
  nor002aa1n02x5               g194(.a(new_n289), .b(new_n285), .o1(\s[26] ));
  and002aa1n24x5               g195(.a(new_n284), .b(new_n283), .o(new_n291));
  nano22aa1d15x5               g196(.a(new_n256), .b(new_n291), .c(new_n274), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n209), .c(new_n145), .d(new_n194), .o1(new_n293));
  nano22aa1n03x5               g198(.a(new_n240), .b(new_n274), .c(new_n255), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[26] ), .b(\b[25] ), .c(new_n286), .carry(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  oaoi13aa1n09x5               g201(.a(new_n296), .b(new_n291), .c(new_n294), .d(new_n278), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  xnbna2aa1n03x5               g203(.a(new_n298), .b(new_n293), .c(new_n297), .out0(\s[27] ));
  norp02aa1n02x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  inv040aa1n03x5               g205(.a(new_n300), .o1(new_n301));
  aobi12aa1n03x5               g206(.a(new_n298), .b(new_n293), .c(new_n297), .out0(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[27] ), .b(\a[28] ), .out0(new_n303));
  nano22aa1n03x5               g208(.a(new_n302), .b(new_n301), .c(new_n303), .out0(new_n304));
  nanp02aa1n09x5               g209(.a(new_n195), .b(new_n200), .o1(new_n305));
  nona32aa1n03x5               g210(.a(new_n257), .b(new_n271), .c(new_n246), .d(new_n244), .out0(new_n306));
  inv040aa1n06x5               g211(.a(new_n278), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n291), .o1(new_n308));
  aoai13aa1n06x5               g213(.a(new_n295), .b(new_n308), .c(new_n306), .d(new_n307), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n298), .b(new_n309), .c(new_n305), .d(new_n292), .o1(new_n310));
  tech160nm_fiaoi012aa1n02p5x5 g215(.a(new_n303), .b(new_n310), .c(new_n301), .o1(new_n311));
  norp02aa1n03x5               g216(.a(new_n311), .b(new_n304), .o1(\s[28] ));
  norb02aa1n02x5               g217(.a(new_n298), .b(new_n303), .out0(new_n313));
  aobi12aa1n03x5               g218(.a(new_n313), .b(new_n293), .c(new_n297), .out0(new_n314));
  oao003aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n301), .carry(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[28] ), .b(\a[29] ), .out0(new_n316));
  nano22aa1n03x5               g221(.a(new_n314), .b(new_n315), .c(new_n316), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n313), .b(new_n309), .c(new_n305), .d(new_n292), .o1(new_n318));
  aoi012aa1n02x7               g223(.a(new_n316), .b(new_n318), .c(new_n315), .o1(new_n319));
  norp02aa1n03x5               g224(.a(new_n319), .b(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g225(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g226(.a(new_n298), .b(new_n316), .c(new_n303), .out0(new_n322));
  aoai13aa1n02x5               g227(.a(new_n322), .b(new_n309), .c(new_n305), .d(new_n292), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .carry(new_n324));
  xnrc02aa1n02x5               g229(.a(\b[29] ), .b(\a[30] ), .out0(new_n325));
  tech160nm_fiaoi012aa1n02p5x5 g230(.a(new_n325), .b(new_n323), .c(new_n324), .o1(new_n326));
  aobi12aa1n03x5               g231(.a(new_n322), .b(new_n293), .c(new_n297), .out0(new_n327));
  nano22aa1n03x5               g232(.a(new_n327), .b(new_n324), .c(new_n325), .out0(new_n328));
  norp02aa1n03x5               g233(.a(new_n326), .b(new_n328), .o1(\s[30] ));
  norb02aa1n03x4               g234(.a(new_n322), .b(new_n325), .out0(new_n330));
  aobi12aa1n03x5               g235(.a(new_n330), .b(new_n293), .c(new_n297), .out0(new_n331));
  oao003aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(new_n324), .carry(new_n332));
  xnrc02aa1n02x5               g237(.a(\b[30] ), .b(\a[31] ), .out0(new_n333));
  nano22aa1n03x5               g238(.a(new_n331), .b(new_n332), .c(new_n333), .out0(new_n334));
  aoai13aa1n03x5               g239(.a(new_n330), .b(new_n309), .c(new_n305), .d(new_n292), .o1(new_n335));
  tech160nm_fiaoi012aa1n02p5x5 g240(.a(new_n333), .b(new_n335), .c(new_n332), .o1(new_n336));
  norp02aa1n03x5               g241(.a(new_n336), .b(new_n334), .o1(\s[31] ));
  xnrb03aa1n02x5               g242(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g243(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n339));
  xorb03aa1n02x5               g244(.a(new_n339), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g245(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oao003aa1n02x5               g246(.a(new_n123), .b(new_n124), .c(new_n109), .carry(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g248(.a(new_n112), .b(new_n113), .out0(new_n344));
  aoai13aa1n06x5               g249(.a(new_n344), .b(new_n115), .c(new_n342), .d(new_n116), .o1(new_n345));
  aoi112aa1n02x5               g250(.a(new_n344), .b(new_n115), .c(new_n342), .d(new_n116), .o1(new_n346));
  norb02aa1n02x5               g251(.a(new_n345), .b(new_n346), .out0(\s[7] ));
  nanb02aa1n02x5               g252(.a(new_n110), .b(new_n111), .out0(new_n348));
  inv000aa1d42x5               g253(.a(new_n113), .o1(new_n349));
  xobna2aa1n03x5               g254(.a(new_n348), .b(new_n345), .c(new_n349), .out0(\s[8] ));
  xnbna2aa1n03x5               g255(.a(new_n128), .b(new_n144), .c(new_n140), .out0(\s[9] ));
endmodule


