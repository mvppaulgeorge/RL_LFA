// Benchmark "adder" written by ABC on Wed Jul 17 14:47:19 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n177, new_n178, new_n179, new_n180, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n309, new_n310,
    new_n313, new_n315, new_n316, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1n03x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nand22aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  tech160nm_fiaoi012aa1n05x5   g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor002aa1d32x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand22aa1n04x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor042aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand02aa1n03x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  tech160nm_fiaoi012aa1n03p5x5 g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n06x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nanp02aa1n04x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nor022aa1n06x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor022aa1n08x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n09x5               g016(.a(new_n108), .b(new_n111), .c(new_n110), .d(new_n109), .out0(new_n112));
  xnrc02aa1n06x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  tech160nm_fixnrc02aa1n03p5x5 g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  nor043aa1n03x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  nor042aa1d18x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  inv040aa1n03x5               g021(.a(new_n116), .o1(new_n117));
  oaoi03aa1n06x5               g022(.a(\a[6] ), .b(\b[5] ), .c(new_n117), .o1(new_n118));
  oai012aa1n02x5               g023(.a(new_n108), .b(new_n110), .c(new_n109), .o1(new_n119));
  oaib12aa1n03x5               g024(.a(new_n119), .b(new_n112), .c(new_n118), .out0(new_n120));
  tech160nm_fiaoi012aa1n05x5   g025(.a(new_n120), .b(new_n107), .c(new_n115), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .c(new_n121), .o1(new_n122));
  xorb03aa1n02x5               g027(.a(new_n122), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand22aa1n04x5               g028(.a(new_n107), .b(new_n115), .o1(new_n124));
  oa0012aa1n02x5               g029(.a(new_n108), .b(new_n109), .c(new_n110), .o(new_n125));
  aoib12aa1n04x5               g030(.a(new_n125), .b(new_n118), .c(new_n112), .out0(new_n126));
  nor002aa1n16x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand02aa1n06x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor022aa1n16x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nona23aa1n09x5               g035(.a(new_n130), .b(new_n128), .c(new_n127), .d(new_n129), .out0(new_n131));
  oai012aa1d24x5               g036(.a(new_n128), .b(new_n129), .c(new_n127), .o1(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n131), .c(new_n124), .d(new_n126), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n16x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1n08x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  aoi012aa1n02x5               g041(.a(new_n135), .b(new_n133), .c(new_n136), .o1(new_n137));
  xnrb03aa1n02x5               g042(.a(new_n137), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor022aa1n16x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand22aa1n12x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nona23aa1n09x5               g045(.a(new_n140), .b(new_n136), .c(new_n135), .d(new_n139), .out0(new_n141));
  norp02aa1n02x5               g046(.a(new_n141), .b(new_n131), .o1(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n143));
  ao0012aa1n03x7               g048(.a(new_n139), .b(new_n135), .c(new_n140), .o(new_n144));
  oabi12aa1n18x5               g049(.a(new_n144), .b(new_n141), .c(new_n132), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  nanp02aa1n03x5               g051(.a(new_n143), .b(new_n146), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanp02aa1n04x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nand02aa1n06x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nano23aa1d18x5               g059(.a(new_n149), .b(new_n153), .c(new_n154), .d(new_n150), .out0(new_n155));
  oaih12aa1n12x5               g060(.a(new_n154), .b(new_n153), .c(new_n149), .o1(new_n156));
  aob012aa1n03x5               g061(.a(new_n156), .b(new_n147), .c(new_n155), .out0(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n03x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  nand42aa1n02x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nanb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  nor042aa1n02x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  nand42aa1n02x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  nanb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  aoai13aa1n03x5               g070(.a(new_n165), .b(new_n159), .c(new_n157), .d(new_n162), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n156), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n162), .b(new_n167), .c(new_n147), .d(new_n155), .o1(new_n168));
  nona22aa1n02x4               g073(.a(new_n168), .b(new_n165), .c(new_n159), .out0(new_n169));
  nanp02aa1n02x5               g074(.a(new_n166), .b(new_n169), .o1(\s[16] ));
  nano23aa1n06x5               g075(.a(new_n159), .b(new_n163), .c(new_n164), .d(new_n160), .out0(new_n171));
  nona23aa1n09x5               g076(.a(new_n155), .b(new_n171), .c(new_n141), .d(new_n131), .out0(new_n172));
  aoai13aa1n06x5               g077(.a(new_n171), .b(new_n167), .c(new_n145), .d(new_n155), .o1(new_n173));
  aoi012aa1n02x5               g078(.a(new_n163), .b(new_n159), .c(new_n164), .o1(new_n174));
  oai112aa1n06x5               g079(.a(new_n173), .b(new_n174), .c(new_n121), .d(new_n172), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  aoi012aa1n12x5               g081(.a(new_n172), .b(new_n124), .c(new_n126), .o1(new_n177));
  inv040aa1n02x5               g082(.a(new_n171), .o1(new_n178));
  inv030aa1n04x5               g083(.a(new_n132), .o1(new_n179));
  nano23aa1n03x5               g084(.a(new_n135), .b(new_n139), .c(new_n140), .d(new_n136), .out0(new_n180));
  aoai13aa1n06x5               g085(.a(new_n155), .b(new_n144), .c(new_n180), .d(new_n179), .o1(new_n181));
  aoai13aa1n06x5               g086(.a(new_n174), .b(new_n178), .c(new_n181), .d(new_n156), .o1(new_n182));
  nor042aa1n06x5               g087(.a(new_n182), .b(new_n177), .o1(new_n183));
  oaoi03aa1n03x5               g088(.a(\a[17] ), .b(\b[16] ), .c(new_n183), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  inv020aa1n04x5               g091(.a(\a[18] ), .o1(new_n187));
  xroi22aa1d06x4               g092(.a(new_n186), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n188));
  oai012aa1n06x5               g093(.a(new_n188), .b(new_n182), .c(new_n177), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[17] ), .o1(new_n190));
  oaih22aa1d12x5               g095(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n191));
  oaib12aa1n18x5               g096(.a(new_n191), .b(new_n190), .c(\a[18] ), .out0(new_n192));
  nor042aa1n06x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  tech160nm_finand02aa1n05x5   g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  xnbna2aa1n03x5               g101(.a(new_n196), .b(new_n189), .c(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n02x5               g103(.a(new_n189), .b(new_n192), .o1(new_n199));
  nor002aa1n06x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  tech160nm_finand02aa1n05x5   g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  aoai13aa1n03x5               g107(.a(new_n202), .b(new_n193), .c(new_n199), .d(new_n196), .o1(new_n203));
  inv040aa1d30x5               g108(.a(new_n192), .o1(new_n204));
  aoai13aa1n03x5               g109(.a(new_n196), .b(new_n204), .c(new_n175), .d(new_n188), .o1(new_n205));
  nona22aa1n02x4               g110(.a(new_n205), .b(new_n202), .c(new_n193), .out0(new_n206));
  nanp02aa1n03x5               g111(.a(new_n203), .b(new_n206), .o1(\s[20] ));
  nano23aa1d15x5               g112(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n208));
  nand22aa1n09x5               g113(.a(new_n188), .b(new_n208), .o1(new_n209));
  nona23aa1n09x5               g114(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n210));
  tech160nm_fiaoi012aa1n04x5   g115(.a(new_n200), .b(new_n193), .c(new_n201), .o1(new_n211));
  tech160nm_fioai012aa1n05x5   g116(.a(new_n211), .b(new_n210), .c(new_n192), .o1(new_n212));
  oabi12aa1n06x5               g117(.a(new_n212), .b(new_n183), .c(new_n209), .out0(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  xorc02aa1n02x5               g120(.a(\a[21] ), .b(\b[20] ), .out0(new_n216));
  xorc02aa1n02x5               g121(.a(\a[22] ), .b(\b[21] ), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n218), .b(new_n215), .c(new_n213), .d(new_n216), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n209), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n216), .b(new_n212), .c(new_n175), .d(new_n220), .o1(new_n221));
  nona22aa1n02x4               g126(.a(new_n221), .b(new_n218), .c(new_n215), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n219), .b(new_n222), .o1(\s[22] ));
  inv000aa1d42x5               g128(.a(\a[21] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\a[22] ), .o1(new_n225));
  xroi22aa1d06x4               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  and003aa1n06x5               g131(.a(new_n188), .b(new_n226), .c(new_n208), .o(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  inv000aa1n02x5               g133(.a(new_n211), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n226), .b(new_n229), .c(new_n208), .d(new_n204), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oaoi03aa1n12x5               g136(.a(new_n225), .b(new_n231), .c(new_n215), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(new_n230), .b(new_n232), .o1(new_n233));
  oabi12aa1n06x5               g138(.a(new_n233), .b(new_n183), .c(new_n228), .out0(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  tech160nm_fixorc02aa1n03p5x5 g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xnrc02aa1n03x5               g142(.a(\b[23] ), .b(\a[24] ), .out0(new_n238));
  aoai13aa1n03x5               g143(.a(new_n238), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n237), .b(new_n233), .c(new_n175), .d(new_n227), .o1(new_n240));
  nona22aa1n02x4               g145(.a(new_n240), .b(new_n238), .c(new_n236), .out0(new_n241));
  nanp02aa1n03x5               g146(.a(new_n239), .b(new_n241), .o1(\s[24] ));
  norb02aa1n02x5               g147(.a(new_n237), .b(new_n238), .out0(new_n243));
  nano22aa1n03x7               g148(.a(new_n209), .b(new_n243), .c(new_n226), .out0(new_n244));
  oaih12aa1n02x5               g149(.a(new_n244), .b(new_n182), .c(new_n177), .o1(new_n245));
  inv000aa1n02x5               g150(.a(new_n243), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n247));
  oab012aa1n02x4               g152(.a(new_n247), .b(\a[24] ), .c(\b[23] ), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n246), .c(new_n230), .d(new_n232), .o1(new_n249));
  nanb02aa1n03x5               g154(.a(new_n249), .b(new_n245), .out0(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  tech160nm_fixorc02aa1n02p5x5 g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  xorc02aa1n12x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n255), .b(new_n252), .c(new_n250), .d(new_n253), .o1(new_n256));
  aoai13aa1n03x5               g161(.a(new_n253), .b(new_n249), .c(new_n175), .d(new_n244), .o1(new_n257));
  nona22aa1n02x4               g162(.a(new_n257), .b(new_n255), .c(new_n252), .out0(new_n258));
  nanp02aa1n02x5               g163(.a(new_n256), .b(new_n258), .o1(\s[26] ));
  and002aa1n06x5               g164(.a(new_n254), .b(new_n253), .o(new_n260));
  inv030aa1n02x5               g165(.a(new_n260), .o1(new_n261));
  nano23aa1d12x5               g166(.a(new_n209), .b(new_n261), .c(new_n243), .d(new_n226), .out0(new_n262));
  oai012aa1n12x5               g167(.a(new_n262), .b(new_n182), .c(new_n177), .o1(new_n263));
  orn002aa1n02x5               g168(.a(\a[25] ), .b(\b[24] ), .o(new_n264));
  oao003aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .c(new_n264), .carry(new_n265));
  aobi12aa1n09x5               g170(.a(new_n265), .b(new_n249), .c(new_n260), .out0(new_n266));
  xorc02aa1n02x5               g171(.a(\a[27] ), .b(\b[26] ), .out0(new_n267));
  xnbna2aa1n03x5               g172(.a(new_n267), .b(new_n266), .c(new_n263), .out0(\s[27] ));
  tech160nm_finand02aa1n03p5x5 g173(.a(new_n266), .b(new_n263), .o1(new_n269));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  norp02aa1n02x5               g175(.a(\b[27] ), .b(\a[28] ), .o1(new_n271));
  nand42aa1n03x5               g176(.a(\b[27] ), .b(\a[28] ), .o1(new_n272));
  norb02aa1n03x5               g177(.a(new_n272), .b(new_n271), .out0(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n270), .c(new_n269), .d(new_n267), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n232), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n243), .b(new_n276), .c(new_n212), .d(new_n226), .o1(new_n277));
  aoai13aa1n04x5               g182(.a(new_n265), .b(new_n261), .c(new_n277), .d(new_n248), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n267), .b(new_n278), .c(new_n175), .d(new_n262), .o1(new_n279));
  nona22aa1n03x5               g184(.a(new_n279), .b(new_n274), .c(new_n270), .out0(new_n280));
  nanp02aa1n03x5               g185(.a(new_n275), .b(new_n280), .o1(\s[28] ));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  norb02aa1n02x5               g187(.a(new_n267), .b(new_n274), .out0(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n278), .c(new_n175), .d(new_n262), .o1(new_n284));
  oai012aa1n02x5               g189(.a(new_n272), .b(new_n271), .c(new_n270), .o1(new_n285));
  aoi012aa1n03x5               g190(.a(new_n282), .b(new_n284), .c(new_n285), .o1(new_n286));
  aobi12aa1n03x5               g191(.a(new_n283), .b(new_n266), .c(new_n263), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n282), .c(new_n285), .out0(new_n288));
  nor002aa1n02x5               g193(.a(new_n286), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g195(.a(\b[29] ), .b(\a[30] ), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n282), .b(new_n267), .c(new_n273), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n278), .c(new_n175), .d(new_n262), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  aoi012aa1n03x5               g199(.a(new_n291), .b(new_n293), .c(new_n294), .o1(new_n295));
  aobi12aa1n03x5               g200(.a(new_n292), .b(new_n266), .c(new_n263), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n291), .c(new_n294), .out0(new_n297));
  nor002aa1n02x5               g202(.a(new_n295), .b(new_n297), .o1(\s[30] ));
  xnrc02aa1n02x5               g203(.a(\b[30] ), .b(\a[31] ), .out0(new_n299));
  nano23aa1n02x4               g204(.a(new_n291), .b(new_n282), .c(new_n267), .d(new_n273), .out0(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n278), .c(new_n175), .d(new_n262), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n302));
  aoi012aa1n03x5               g207(.a(new_n299), .b(new_n301), .c(new_n302), .o1(new_n303));
  aobi12aa1n06x5               g208(.a(new_n300), .b(new_n266), .c(new_n263), .out0(new_n304));
  nano22aa1n03x5               g209(.a(new_n304), .b(new_n299), .c(new_n302), .out0(new_n305));
  nor002aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[31] ));
  xnrb03aa1n02x5               g211(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  inv000aa1d42x5               g212(.a(new_n101), .o1(new_n308));
  nona22aa1n02x4               g213(.a(new_n104), .b(new_n100), .c(new_n103), .out0(new_n309));
  aoi012aa1n02x5               g214(.a(new_n103), .b(new_n308), .c(new_n102), .o1(new_n310));
  aoi022aa1n02x5               g215(.a(new_n107), .b(new_n308), .c(new_n309), .d(new_n310), .o1(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g217(.a(new_n114), .b(new_n107), .out0(new_n313));
  xobna2aa1n03x5               g218(.a(new_n113), .b(new_n313), .c(new_n117), .out0(\s[6] ));
  nanb02aa1n02x5               g219(.a(new_n110), .b(new_n111), .out0(new_n315));
  nanp02aa1n02x5               g220(.a(\b[5] ), .b(\a[6] ), .o1(new_n316));
  nona22aa1n02x4               g221(.a(new_n313), .b(new_n116), .c(new_n113), .out0(new_n317));
  xnbna2aa1n03x5               g222(.a(new_n315), .b(new_n317), .c(new_n316), .out0(\s[7] ));
  aoi013aa1n02x4               g223(.a(new_n110), .b(new_n317), .c(new_n316), .d(new_n111), .o1(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g225(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


