// Benchmark "adder" written by ABC on Wed Jul 17 16:52:25 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n150, new_n151, new_n152, new_n153, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n330, new_n331,
    new_n333, new_n335, new_n337, new_n338, new_n341;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(\b[8] ), .b(new_n98), .out0(new_n99));
  nor042aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nanp02aa1n12x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nanp02aa1n12x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi012aa1n12x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand02aa1d12x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand02aa1d06x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  oai012aa1n12x5               g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai012aa1n12x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand02aa1d24x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1n12x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n08x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  nand02aa1d28x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor002aa1d32x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanb02aa1d24x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  xorc02aa1n12x5               g023(.a(\a[5] ), .b(\b[4] ), .out0(new_n119));
  norb03aa1n12x5               g024(.a(new_n119), .b(new_n115), .c(new_n118), .out0(new_n120));
  inv000aa1d42x5               g025(.a(new_n111), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n112), .o1(new_n122));
  inv000aa1n02x5               g027(.a(new_n113), .o1(new_n123));
  nor042aa1n09x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  aoai13aa1n12x5               g029(.a(new_n114), .b(new_n117), .c(new_n124), .d(new_n116), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n121), .b(new_n122), .c(new_n125), .d(new_n123), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n03x5               g032(.a(new_n127), .b(new_n126), .c(new_n120), .d(new_n110), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n99), .out0(\s[10] ));
  nand42aa1d28x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nor002aa1d24x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  oaih22aa1d12x5               g037(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(new_n128), .b(new_n134), .o1(new_n135));
  nand42aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  inv020aa1n04x5               g041(.a(new_n103), .o1(new_n137));
  nano23aa1n06x5               g042(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n138));
  aobi12aa1n12x5               g043(.a(new_n109), .b(new_n138), .c(new_n137), .out0(new_n139));
  nanb02aa1n12x5               g044(.a(new_n111), .b(new_n112), .out0(new_n140));
  norb02aa1n03x5               g045(.a(new_n114), .b(new_n113), .out0(new_n141));
  nona23aa1n09x5               g046(.a(new_n119), .b(new_n141), .c(new_n140), .d(new_n118), .out0(new_n142));
  tech160nm_finand02aa1n03p5x5 g047(.a(new_n125), .b(new_n123), .o1(new_n143));
  tech160nm_fiaoi012aa1n03p5x5 g048(.a(new_n111), .b(new_n143), .c(new_n112), .o1(new_n144));
  oai012aa1n12x5               g049(.a(new_n144), .b(new_n139), .c(new_n142), .o1(new_n145));
  aoai13aa1n02x5               g050(.a(new_n136), .b(new_n133), .c(new_n145), .d(new_n127), .o1(new_n146));
  nano22aa1n03x7               g051(.a(new_n131), .b(new_n136), .c(new_n130), .out0(new_n147));
  aoi022aa1n02x5               g052(.a(new_n146), .b(new_n132), .c(new_n135), .d(new_n147), .o1(\s[11] ));
  aoi022aa1d24x5               g053(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n149));
  tech160nm_fiaoi012aa1n05x5   g054(.a(new_n131), .b(new_n135), .c(new_n149), .o1(new_n150));
  norp02aa1n12x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nand42aa1d28x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  norb02aa1n03x5               g057(.a(new_n152), .b(new_n151), .out0(new_n153));
  xnrc02aa1n02x5               g058(.a(new_n150), .b(new_n153), .out0(\s[12] ));
  nano23aa1n06x5               g059(.a(new_n151), .b(new_n131), .c(new_n152), .d(new_n130), .out0(new_n155));
  and003aa1n02x5               g060(.a(new_n155), .b(new_n127), .c(new_n97), .o(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n126), .c(new_n120), .d(new_n110), .o1(new_n157));
  oa0012aa1n06x5               g062(.a(new_n152), .b(new_n151), .c(new_n131), .o(new_n158));
  aoi013aa1n06x4               g063(.a(new_n158), .b(new_n147), .c(new_n153), .d(new_n133), .o1(new_n159));
  nor042aa1d18x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nand42aa1d28x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n157), .c(new_n159), .out0(\s[13] ));
  nanp02aa1n02x5               g068(.a(new_n157), .b(new_n159), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n160), .b(new_n164), .c(new_n162), .o1(new_n165));
  xnrb03aa1n02x5               g070(.a(new_n165), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nona23aa1n09x5               g071(.a(new_n149), .b(new_n152), .c(new_n151), .d(new_n131), .out0(new_n167));
  oabi12aa1n18x5               g072(.a(new_n158), .b(new_n167), .c(new_n134), .out0(new_n168));
  nor042aa1n09x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand42aa1d28x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nano23aa1n09x5               g075(.a(new_n160), .b(new_n169), .c(new_n170), .d(new_n161), .out0(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n168), .c(new_n145), .d(new_n156), .o1(new_n172));
  aoi012aa1n02x5               g077(.a(new_n169), .b(new_n160), .c(new_n170), .o1(new_n173));
  nor042aa1d18x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nand02aa1d28x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n172), .c(new_n173), .out0(\s[15] ));
  nand42aa1n02x5               g082(.a(new_n172), .b(new_n173), .o1(new_n178));
  nor042aa1d18x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nand02aa1d28x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  aoai13aa1n03x5               g086(.a(new_n181), .b(new_n174), .c(new_n178), .d(new_n175), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(new_n178), .b(new_n176), .o1(new_n183));
  nona22aa1n02x4               g088(.a(new_n183), .b(new_n181), .c(new_n174), .out0(new_n184));
  nanp02aa1n02x5               g089(.a(new_n184), .b(new_n182), .o1(\s[16] ));
  nano23aa1n09x5               g090(.a(new_n174), .b(new_n179), .c(new_n180), .d(new_n175), .out0(new_n186));
  nand02aa1d04x5               g091(.a(new_n186), .b(new_n171), .o1(new_n187));
  nano32aa1d12x5               g092(.a(new_n187), .b(new_n155), .c(new_n127), .d(new_n97), .out0(new_n188));
  aoai13aa1n12x5               g093(.a(new_n188), .b(new_n126), .c(new_n120), .d(new_n110), .o1(new_n189));
  inv000aa1d42x5               g094(.a(new_n174), .o1(new_n190));
  inv000aa1d42x5               g095(.a(new_n179), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n180), .o1(new_n192));
  aoai13aa1n04x5               g097(.a(new_n175), .b(new_n169), .c(new_n160), .d(new_n170), .o1(new_n193));
  aoai13aa1n06x5               g098(.a(new_n191), .b(new_n192), .c(new_n193), .d(new_n190), .o1(new_n194));
  aoib12aa1n09x5               g099(.a(new_n194), .b(new_n168), .c(new_n187), .out0(new_n195));
  nanp02aa1n06x5               g100(.a(new_n189), .b(new_n195), .o1(new_n196));
  nor002aa1d32x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  nand42aa1d28x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  aoi113aa1n02x5               g104(.a(new_n194), .b(new_n199), .c(new_n168), .d(new_n171), .e(new_n186), .o1(new_n200));
  aoi022aa1n02x5               g105(.a(new_n196), .b(new_n199), .c(new_n200), .d(new_n189), .o1(\s[17] ));
  tech160nm_fiaoi012aa1n05x5   g106(.a(new_n197), .b(new_n196), .c(new_n199), .o1(new_n202));
  xnrb03aa1n03x5               g107(.a(new_n202), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  oabi12aa1n06x5               g108(.a(new_n194), .b(new_n187), .c(new_n159), .out0(new_n204));
  nor002aa1d24x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nand42aa1d28x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nano23aa1d15x5               g111(.a(new_n197), .b(new_n205), .c(new_n206), .d(new_n198), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n204), .c(new_n145), .d(new_n188), .o1(new_n208));
  oa0012aa1n02x5               g113(.a(new_n206), .b(new_n205), .c(new_n197), .o(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  nor042aa1d18x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nand22aa1n12x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  norb02aa1n09x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n213), .b(new_n208), .c(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n03x5               g120(.a(new_n208), .b(new_n210), .o1(new_n216));
  nor002aa1d32x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nand02aa1d28x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nanb02aa1d24x5               g123(.a(new_n217), .b(new_n218), .out0(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n211), .c(new_n216), .d(new_n212), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n213), .b(new_n209), .c(new_n196), .d(new_n207), .o1(new_n221));
  nona22aa1n03x5               g126(.a(new_n221), .b(new_n219), .c(new_n211), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n220), .b(new_n222), .o1(\s[20] ));
  nanb03aa1d18x5               g128(.a(new_n217), .b(new_n218), .c(new_n212), .out0(new_n224));
  oai122aa1n12x5               g129(.a(new_n206), .b(new_n205), .c(new_n197), .d(\b[18] ), .e(\a[19] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n217), .o1(new_n226));
  aob012aa1d15x5               g131(.a(new_n226), .b(new_n211), .c(new_n218), .out0(new_n227));
  oabi12aa1n18x5               g132(.a(new_n227), .b(new_n225), .c(new_n224), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  nanb03aa1d18x5               g134(.a(new_n219), .b(new_n207), .c(new_n213), .out0(new_n230));
  aoai13aa1n04x5               g135(.a(new_n229), .b(new_n230), .c(new_n189), .d(new_n195), .o1(new_n231));
  nor042aa1n09x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nand42aa1n08x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n230), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n234), .b(new_n196), .c(new_n235), .o1(new_n236));
  aoi022aa1n02x5               g141(.a(new_n236), .b(new_n229), .c(new_n231), .d(new_n234), .o1(\s[21] ));
  nor042aa1n06x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nand42aa1n06x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  nanb02aa1n02x5               g144(.a(new_n238), .b(new_n239), .out0(new_n240));
  aoai13aa1n03x5               g145(.a(new_n240), .b(new_n232), .c(new_n231), .d(new_n234), .o1(new_n241));
  aoi112aa1n03x5               g146(.a(new_n232), .b(new_n240), .c(new_n231), .d(new_n233), .o1(new_n242));
  nanb02aa1n03x5               g147(.a(new_n242), .b(new_n241), .out0(\s[22] ));
  nano23aa1d15x5               g148(.a(new_n232), .b(new_n238), .c(new_n239), .d(new_n233), .out0(new_n244));
  nano32aa1n02x5               g149(.a(new_n219), .b(new_n244), .c(new_n207), .d(new_n213), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n204), .c(new_n145), .d(new_n188), .o1(new_n246));
  nano22aa1n03x7               g151(.a(new_n217), .b(new_n212), .c(new_n218), .out0(new_n247));
  tech160nm_fioai012aa1n03p5x5 g152(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .o1(new_n248));
  oab012aa1n06x5               g153(.a(new_n248), .b(new_n197), .c(new_n205), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n244), .b(new_n227), .c(new_n249), .d(new_n247), .o1(new_n250));
  oa0012aa1n03x5               g155(.a(new_n239), .b(new_n238), .c(new_n232), .o(new_n251));
  inv030aa1n02x5               g156(.a(new_n251), .o1(new_n252));
  nand22aa1n06x5               g157(.a(new_n250), .b(new_n252), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  xorc02aa1n12x5               g159(.a(\a[23] ), .b(\b[22] ), .out0(new_n255));
  xnbna2aa1n03x5               g160(.a(new_n255), .b(new_n246), .c(new_n254), .out0(\s[23] ));
  nanp02aa1n02x5               g161(.a(new_n246), .b(new_n254), .o1(new_n257));
  norp02aa1n02x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  xnrc02aa1n12x5               g163(.a(\b[23] ), .b(\a[24] ), .out0(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n258), .c(new_n257), .d(new_n255), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n255), .b(new_n253), .c(new_n196), .d(new_n245), .o1(new_n261));
  nona22aa1n02x5               g166(.a(new_n261), .b(new_n259), .c(new_n258), .out0(new_n262));
  nanp02aa1n03x5               g167(.a(new_n260), .b(new_n262), .o1(\s[24] ));
  norb02aa1n12x5               g168(.a(new_n255), .b(new_n259), .out0(new_n264));
  nand02aa1n02x5               g169(.a(new_n245), .b(new_n264), .o1(new_n265));
  oai022aa1n02x5               g170(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n266));
  aob012aa1n02x5               g171(.a(new_n266), .b(\b[23] ), .c(\a[24] ), .out0(new_n267));
  aobi12aa1n06x5               g172(.a(new_n267), .b(new_n253), .c(new_n264), .out0(new_n268));
  aoai13aa1n04x5               g173(.a(new_n268), .b(new_n265), .c(new_n189), .d(new_n195), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g175(.a(\b[24] ), .b(\a[25] ), .o1(new_n271));
  xorc02aa1n12x5               g176(.a(\a[25] ), .b(\b[24] ), .out0(new_n272));
  xorc02aa1n12x5               g177(.a(\a[26] ), .b(\b[25] ), .out0(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n271), .c(new_n269), .d(new_n272), .o1(new_n275));
  aoi112aa1n03x5               g180(.a(new_n271), .b(new_n274), .c(new_n269), .d(new_n272), .o1(new_n276));
  nanb02aa1n03x5               g181(.a(new_n276), .b(new_n275), .out0(\s[26] ));
  nand02aa1n06x5               g182(.a(new_n273), .b(new_n272), .o1(new_n278));
  nano23aa1n06x5               g183(.a(new_n230), .b(new_n278), .c(new_n264), .d(new_n244), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n204), .c(new_n145), .d(new_n188), .o1(new_n280));
  inv000aa1n02x5               g185(.a(new_n264), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n267), .b(new_n281), .c(new_n250), .d(new_n252), .o1(new_n282));
  nanp02aa1n02x5               g187(.a(\b[25] ), .b(\a[26] ), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n278), .o1(new_n284));
  oai022aa1n02x5               g189(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n285));
  aoi022aa1n09x5               g190(.a(new_n282), .b(new_n284), .c(new_n283), .d(new_n285), .o1(new_n286));
  tech160nm_finand02aa1n03p5x5 g191(.a(new_n286), .b(new_n280), .o1(new_n287));
  xorc02aa1n12x5               g192(.a(\a[27] ), .b(\b[26] ), .out0(new_n288));
  aoi122aa1n02x5               g193(.a(new_n288), .b(new_n283), .c(new_n285), .d(new_n282), .e(new_n284), .o1(new_n289));
  aoi022aa1n02x5               g194(.a(new_n287), .b(new_n288), .c(new_n289), .d(new_n280), .o1(\s[27] ));
  norp02aa1n02x5               g195(.a(\b[26] ), .b(\a[27] ), .o1(new_n291));
  xnrc02aa1n12x5               g196(.a(\b[27] ), .b(\a[28] ), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n287), .d(new_n288), .o1(new_n293));
  aobi12aa1n06x5               g198(.a(new_n279), .b(new_n189), .c(new_n195), .out0(new_n294));
  aoai13aa1n04x5               g199(.a(new_n264), .b(new_n251), .c(new_n228), .d(new_n244), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(new_n285), .b(new_n283), .o1(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n278), .c(new_n295), .d(new_n267), .o1(new_n297));
  oaih12aa1n02x5               g202(.a(new_n288), .b(new_n297), .c(new_n294), .o1(new_n298));
  nona22aa1n02x5               g203(.a(new_n298), .b(new_n292), .c(new_n291), .out0(new_n299));
  nanp02aa1n03x5               g204(.a(new_n293), .b(new_n299), .o1(\s[28] ));
  norb02aa1n15x5               g205(.a(new_n288), .b(new_n292), .out0(new_n301));
  oaih12aa1n02x5               g206(.a(new_n301), .b(new_n297), .c(new_n294), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n301), .o1(new_n303));
  orn002aa1n02x5               g208(.a(\a[27] ), .b(\b[26] ), .o(new_n304));
  oao003aa1n03x5               g209(.a(\a[28] ), .b(\b[27] ), .c(new_n304), .carry(new_n305));
  aoai13aa1n02x7               g210(.a(new_n305), .b(new_n303), .c(new_n286), .d(new_n280), .o1(new_n306));
  xorc02aa1n12x5               g211(.a(\a[29] ), .b(\b[28] ), .out0(new_n307));
  norb02aa1n02x5               g212(.a(new_n305), .b(new_n307), .out0(new_n308));
  aoi022aa1n03x5               g213(.a(new_n306), .b(new_n307), .c(new_n302), .d(new_n308), .o1(\s[29] ));
  xorb03aa1n02x5               g214(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g215(.a(new_n292), .b(new_n288), .c(new_n307), .out0(new_n311));
  oaih12aa1n02x5               g216(.a(new_n311), .b(new_n297), .c(new_n294), .o1(new_n312));
  inv000aa1n02x5               g217(.a(new_n311), .o1(new_n313));
  oaoi03aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .c(new_n305), .o1(new_n314));
  inv000aa1n03x5               g219(.a(new_n314), .o1(new_n315));
  aoai13aa1n02x7               g220(.a(new_n315), .b(new_n313), .c(new_n286), .d(new_n280), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[30] ), .b(\b[29] ), .out0(new_n317));
  and002aa1n02x5               g222(.a(\b[28] ), .b(\a[29] ), .o(new_n318));
  oabi12aa1n02x5               g223(.a(new_n317), .b(\a[29] ), .c(\b[28] ), .out0(new_n319));
  oab012aa1n02x4               g224(.a(new_n319), .b(new_n305), .c(new_n318), .out0(new_n320));
  aoi022aa1n03x5               g225(.a(new_n316), .b(new_n317), .c(new_n312), .d(new_n320), .o1(\s[30] ));
  nand03aa1n02x5               g226(.a(new_n301), .b(new_n307), .c(new_n317), .o1(new_n322));
  oabi12aa1n03x5               g227(.a(new_n322), .b(new_n297), .c(new_n294), .out0(new_n323));
  xorc02aa1n02x5               g228(.a(\a[31] ), .b(\b[30] ), .out0(new_n324));
  oao003aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .c(new_n315), .carry(new_n325));
  norb02aa1n02x5               g230(.a(new_n325), .b(new_n324), .out0(new_n326));
  aoai13aa1n02x7               g231(.a(new_n325), .b(new_n322), .c(new_n286), .d(new_n280), .o1(new_n327));
  aoi022aa1n03x5               g232(.a(new_n327), .b(new_n324), .c(new_n323), .d(new_n326), .o1(\s[31] ));
  xnrb03aa1n02x5               g233(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  norb02aa1n02x5               g234(.a(new_n105), .b(new_n104), .out0(new_n330));
  aoi112aa1n02x5               g235(.a(new_n106), .b(new_n330), .c(new_n137), .d(new_n107), .o1(new_n331));
  aoib12aa1n02x5               g236(.a(new_n331), .b(new_n110), .c(new_n104), .out0(\s[4] ));
  nanp02aa1n02x5               g237(.a(new_n138), .b(new_n137), .o1(new_n333));
  xnbna2aa1n03x5               g238(.a(new_n119), .b(new_n333), .c(new_n109), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g239(.a(\a[5] ), .b(\b[4] ), .c(new_n139), .o1(new_n335));
  xorb03aa1n02x5               g240(.a(new_n335), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n03x5               g241(.a(new_n141), .b(new_n117), .c(new_n335), .d(new_n116), .o1(new_n337));
  aoi112aa1n02x5               g242(.a(new_n117), .b(new_n141), .c(new_n335), .d(new_n116), .o1(new_n338));
  norb02aa1n02x5               g243(.a(new_n337), .b(new_n338), .out0(\s[7] ));
  xobna2aa1n03x5               g244(.a(new_n140), .b(new_n337), .c(new_n123), .out0(\s[8] ));
  aoi112aa1n02x5               g245(.a(new_n126), .b(new_n127), .c(new_n120), .d(new_n110), .o1(new_n341));
  aoi012aa1n02x5               g246(.a(new_n341), .b(new_n145), .c(new_n127), .o1(\s[9] ));
endmodule


