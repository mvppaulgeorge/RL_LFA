// Benchmark "adder" written by ABC on Wed Jul 17 15:23:16 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n322, new_n324,
    new_n325, new_n328, new_n329, new_n330, new_n332, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[9] ), .o1(new_n99));
  nanb02aa1d36x5               g004(.a(\b[8] ), .b(new_n99), .out0(new_n100));
  inv000aa1d42x5               g005(.a(\a[2] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[1] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  oao003aa1n02x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .carry(new_n104));
  nand42aa1n03x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor042aa1n09x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nor042aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nano23aa1n06x5               g013(.a(new_n107), .b(new_n106), .c(new_n108), .d(new_n105), .out0(new_n109));
  aoi012aa1n03x5               g014(.a(new_n106), .b(new_n107), .c(new_n105), .o1(new_n110));
  aob012aa1n06x5               g015(.a(new_n110), .b(new_n109), .c(new_n104), .out0(new_n111));
  tech160nm_fixorc02aa1n03p5x5 g016(.a(\a[8] ), .b(\b[7] ), .out0(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  tech160nm_fixorc02aa1n03p5x5 g021(.a(\a[6] ), .b(\b[5] ), .out0(new_n117));
  nano23aa1n03x7               g022(.a(new_n116), .b(new_n115), .c(new_n117), .d(new_n112), .out0(new_n118));
  and002aa1n24x5               g023(.a(\b[5] ), .b(\a[6] ), .o(new_n119));
  inv000aa1d42x5               g024(.a(new_n119), .o1(new_n120));
  oaih22aa1d12x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  aoi013aa1n02x4               g026(.a(new_n113), .b(new_n120), .c(new_n121), .d(new_n114), .o1(new_n122));
  oaoi03aa1n06x5               g027(.a(\a[8] ), .b(\b[7] ), .c(new_n122), .o1(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(new_n111), .d(new_n118), .o1(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n98), .b(new_n125), .c(new_n100), .out0(\s[10] ));
  inv000aa1d42x5               g031(.a(new_n100), .o1(new_n127));
  aobi12aa1n12x5               g032(.a(new_n110), .b(new_n109), .c(new_n104), .out0(new_n128));
  nona23aa1n09x5               g033(.a(new_n112), .b(new_n117), .c(new_n116), .d(new_n115), .out0(new_n129));
  inv000aa1d42x5               g034(.a(\a[8] ), .o1(new_n130));
  inv000aa1d42x5               g035(.a(\b[7] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n113), .o1(new_n132));
  nanb03aa1n03x5               g037(.a(new_n119), .b(new_n121), .c(new_n114), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  tech160nm_fioaoi03aa1n03p5x5 g039(.a(new_n130), .b(new_n131), .c(new_n134), .o1(new_n135));
  oaih12aa1n12x5               g040(.a(new_n135), .b(new_n128), .c(new_n129), .o1(new_n136));
  aoai13aa1n06x5               g041(.a(new_n98), .b(new_n127), .c(new_n136), .d(new_n124), .o1(new_n137));
  oaoi03aa1n12x5               g042(.a(\a[10] ), .b(\b[9] ), .c(new_n100), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n138), .o1(new_n139));
  tech160nm_fixnrc02aa1n04x5   g044(.a(\b[10] ), .b(\a[11] ), .out0(new_n140));
  inv000aa1n02x5               g045(.a(new_n140), .o1(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n137), .c(new_n139), .out0(\s[11] ));
  aoai13aa1n02x5               g047(.a(new_n139), .b(new_n97), .c(new_n125), .d(new_n100), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(new_n143), .b(new_n141), .o1(new_n144));
  xorc02aa1n12x5               g049(.a(\a[12] ), .b(\b[11] ), .out0(new_n145));
  nor042aa1n09x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  norp02aa1n02x5               g051(.a(new_n145), .b(new_n146), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n146), .o1(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n140), .c(new_n137), .d(new_n139), .o1(new_n149));
  aoi022aa1n02x5               g054(.a(new_n149), .b(new_n145), .c(new_n144), .d(new_n147), .o1(\s[12] ));
  nona23aa1d18x5               g055(.a(new_n145), .b(new_n124), .c(new_n97), .d(new_n140), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n123), .c(new_n111), .d(new_n118), .o1(new_n153));
  oaoi03aa1n02x5               g058(.a(\a[12] ), .b(\b[11] ), .c(new_n148), .o1(new_n154));
  aoi013aa1n09x5               g059(.a(new_n154), .b(new_n141), .c(new_n138), .d(new_n145), .o1(new_n155));
  xnrc02aa1n12x5               g060(.a(\b[12] ), .b(\a[13] ), .out0(new_n156));
  xobna2aa1n03x5               g061(.a(new_n156), .b(new_n153), .c(new_n155), .out0(\s[13] ));
  nand42aa1n02x5               g062(.a(new_n153), .b(new_n155), .o1(new_n158));
  nanb02aa1n02x5               g063(.a(new_n156), .b(new_n158), .out0(new_n159));
  xnrc02aa1n12x5               g064(.a(\b[13] ), .b(\a[14] ), .out0(new_n160));
  nor042aa1n06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(new_n162));
  inv000aa1d42x5               g067(.a(new_n161), .o1(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n156), .c(new_n153), .d(new_n155), .o1(new_n164));
  aboi22aa1n03x5               g069(.a(new_n160), .b(new_n164), .c(new_n159), .d(new_n162), .out0(\s[14] ));
  inv000aa1n02x5               g070(.a(new_n155), .o1(new_n166));
  nor042aa1n02x5               g071(.a(new_n160), .b(new_n156), .o1(new_n167));
  aoai13aa1n03x5               g072(.a(new_n167), .b(new_n166), .c(new_n136), .d(new_n152), .o1(new_n168));
  tech160nm_fioaoi03aa1n03p5x5 g073(.a(\a[14] ), .b(\b[13] ), .c(new_n163), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  xnrc02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .out0(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n168), .c(new_n170), .out0(\s[15] ));
  aoai13aa1n02x5               g078(.a(new_n172), .b(new_n169), .c(new_n158), .d(new_n167), .o1(new_n174));
  xorc02aa1n02x5               g079(.a(\a[16] ), .b(\b[15] ), .out0(new_n175));
  inv000aa1d42x5               g080(.a(\a[15] ), .o1(new_n176));
  inv000aa1d42x5               g081(.a(\b[14] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(new_n177), .b(new_n176), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n175), .out0(new_n179));
  aoai13aa1n03x5               g084(.a(new_n178), .b(new_n171), .c(new_n168), .d(new_n170), .o1(new_n180));
  aoi022aa1n03x5               g085(.a(new_n180), .b(new_n175), .c(new_n174), .d(new_n179), .o1(\s[16] ));
  inv000aa1d42x5               g086(.a(\a[16] ), .o1(new_n182));
  xroi22aa1d04x5               g087(.a(new_n176), .b(\b[14] ), .c(new_n182), .d(\b[15] ), .out0(new_n183));
  nano22aa1n12x5               g088(.a(new_n151), .b(new_n167), .c(new_n183), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n123), .c(new_n111), .d(new_n118), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(new_n183), .b(new_n167), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\b[15] ), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(\b[13] ), .b(\a[14] ), .o1(new_n188));
  oai022aa1n02x5               g093(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n189));
  oai112aa1n02x5               g094(.a(new_n189), .b(new_n188), .c(new_n177), .d(new_n176), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(new_n190), .b(new_n178), .o1(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n182), .b(new_n187), .c(new_n191), .o1(new_n192));
  oai012aa1d24x5               g097(.a(new_n192), .b(new_n155), .c(new_n186), .o1(new_n193));
  nanb02aa1d24x5               g098(.a(new_n193), .b(new_n185), .out0(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g100(.a(\a[17] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(\b[16] ), .b(new_n196), .out0(new_n197));
  xorc02aa1n02x5               g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n193), .c(new_n136), .d(new_n184), .o1(new_n199));
  xorc02aa1n02x5               g104(.a(\a[18] ), .b(\b[17] ), .out0(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n199), .c(new_n197), .out0(\s[18] ));
  inv000aa1d42x5               g106(.a(\a[18] ), .o1(new_n202));
  xroi22aa1d04x5               g107(.a(new_n196), .b(\b[16] ), .c(new_n202), .d(\b[17] ), .out0(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n193), .c(new_n136), .d(new_n184), .o1(new_n204));
  oai022aa1n02x5               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  oaib12aa1n06x5               g110(.a(new_n205), .b(new_n202), .c(\b[17] ), .out0(new_n206));
  nor002aa1d32x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand42aa1n06x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n204), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n02x5               g117(.a(\a[18] ), .b(\b[17] ), .c(new_n197), .o1(new_n213));
  aoai13aa1n03x5               g118(.a(new_n210), .b(new_n213), .c(new_n194), .d(new_n203), .o1(new_n214));
  nor002aa1d24x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nand22aa1n09x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  aoib12aa1n02x5               g122(.a(new_n207), .b(new_n216), .c(new_n215), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n207), .o1(new_n219));
  aoai13aa1n02x7               g124(.a(new_n219), .b(new_n209), .c(new_n204), .d(new_n206), .o1(new_n220));
  aoi022aa1n03x5               g125(.a(new_n220), .b(new_n217), .c(new_n214), .d(new_n218), .o1(\s[20] ));
  nona23aa1d18x5               g126(.a(new_n216), .b(new_n208), .c(new_n207), .d(new_n215), .out0(new_n222));
  nano22aa1n02x4               g127(.a(new_n222), .b(new_n198), .c(new_n200), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n193), .c(new_n136), .d(new_n184), .o1(new_n224));
  aoi012aa1n06x5               g129(.a(new_n215), .b(new_n207), .c(new_n216), .o1(new_n225));
  oai012aa1d24x5               g130(.a(new_n225), .b(new_n222), .c(new_n206), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  xnrc02aa1n12x5               g132(.a(\b[20] ), .b(\a[21] ), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  xnbna2aa1n03x5               g134(.a(new_n229), .b(new_n224), .c(new_n227), .out0(\s[21] ));
  aoai13aa1n03x5               g135(.a(new_n229), .b(new_n226), .c(new_n194), .d(new_n223), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[21] ), .b(\a[22] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  nor042aa1n03x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n232), .b(new_n234), .out0(new_n235));
  inv000aa1n03x5               g140(.a(new_n234), .o1(new_n236));
  aoai13aa1n02x7               g141(.a(new_n236), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n237));
  aoi022aa1n03x5               g142(.a(new_n237), .b(new_n233), .c(new_n231), .d(new_n235), .o1(\s[22] ));
  nano23aa1n03x7               g143(.a(new_n207), .b(new_n215), .c(new_n216), .d(new_n208), .out0(new_n239));
  nor042aa1n06x5               g144(.a(new_n232), .b(new_n228), .o1(new_n240));
  and003aa1n02x5               g145(.a(new_n203), .b(new_n240), .c(new_n239), .o(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n193), .c(new_n136), .d(new_n184), .o1(new_n242));
  oaoi03aa1n09x5               g147(.a(\a[22] ), .b(\b[21] ), .c(new_n236), .o1(new_n243));
  tech160nm_fiaoi012aa1n05x5   g148(.a(new_n243), .b(new_n226), .c(new_n240), .o1(new_n244));
  xnrc02aa1n12x5               g149(.a(\b[22] ), .b(\a[23] ), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  xnbna2aa1n03x5               g151(.a(new_n246), .b(new_n242), .c(new_n244), .out0(\s[23] ));
  inv000aa1n02x5               g152(.a(new_n244), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n246), .b(new_n248), .c(new_n194), .d(new_n241), .o1(new_n249));
  tech160nm_fixorc02aa1n02p5x5 g154(.a(\a[24] ), .b(\b[23] ), .out0(new_n250));
  nor042aa1n06x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  norp02aa1n02x5               g156(.a(new_n250), .b(new_n251), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n251), .o1(new_n253));
  aoai13aa1n02x7               g158(.a(new_n253), .b(new_n245), .c(new_n242), .d(new_n244), .o1(new_n254));
  aoi022aa1n03x5               g159(.a(new_n254), .b(new_n250), .c(new_n249), .d(new_n252), .o1(\s[24] ));
  norb02aa1n03x5               g160(.a(new_n250), .b(new_n245), .out0(new_n256));
  inv000aa1n03x5               g161(.a(new_n256), .o1(new_n257));
  nano32aa1n02x4               g162(.a(new_n257), .b(new_n203), .c(new_n240), .d(new_n239), .out0(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n193), .c(new_n136), .d(new_n184), .o1(new_n259));
  inv020aa1n04x5               g164(.a(new_n225), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n240), .b(new_n260), .c(new_n239), .d(new_n213), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n243), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[24] ), .b(\b[23] ), .c(new_n253), .carry(new_n263));
  aoai13aa1n12x5               g168(.a(new_n263), .b(new_n257), .c(new_n261), .d(new_n262), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  xorc02aa1n12x5               g170(.a(\a[25] ), .b(\b[24] ), .out0(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n259), .c(new_n265), .out0(\s[25] ));
  aoai13aa1n03x5               g172(.a(new_n266), .b(new_n264), .c(new_n194), .d(new_n258), .o1(new_n268));
  xorc02aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .out0(new_n269));
  nor042aa1n03x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  norp02aa1n02x5               g175(.a(new_n269), .b(new_n270), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n270), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n266), .o1(new_n273));
  aoai13aa1n02x7               g178(.a(new_n272), .b(new_n273), .c(new_n259), .d(new_n265), .o1(new_n274));
  aoi022aa1n03x5               g179(.a(new_n274), .b(new_n269), .c(new_n268), .d(new_n271), .o1(\s[26] ));
  and002aa1n02x5               g180(.a(new_n269), .b(new_n266), .o(new_n276));
  inv000aa1n02x5               g181(.a(new_n276), .o1(new_n277));
  nano32aa1n06x5               g182(.a(new_n277), .b(new_n223), .c(new_n240), .d(new_n256), .out0(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n193), .c(new_n136), .d(new_n184), .o1(new_n279));
  oao003aa1n02x5               g184(.a(\a[26] ), .b(\b[25] ), .c(new_n272), .carry(new_n280));
  inv000aa1d42x5               g185(.a(new_n280), .o1(new_n281));
  aoi012aa1n09x5               g186(.a(new_n281), .b(new_n264), .c(new_n276), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[27] ), .b(\b[26] ), .out0(new_n283));
  xnbna2aa1n06x5               g188(.a(new_n283), .b(new_n282), .c(new_n279), .out0(\s[27] ));
  aoai13aa1n06x5               g189(.a(new_n256), .b(new_n243), .c(new_n226), .d(new_n240), .o1(new_n285));
  aoai13aa1n06x5               g190(.a(new_n280), .b(new_n277), .c(new_n285), .d(new_n263), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n283), .b(new_n286), .c(new_n194), .d(new_n278), .o1(new_n287));
  tech160nm_fixorc02aa1n02p5x5 g192(.a(\a[28] ), .b(\b[27] ), .out0(new_n288));
  norp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n288), .b(new_n289), .o1(new_n290));
  inv000aa1n03x5               g195(.a(new_n289), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n283), .o1(new_n292));
  aoai13aa1n02x7               g197(.a(new_n291), .b(new_n292), .c(new_n282), .d(new_n279), .o1(new_n293));
  aoi022aa1n03x5               g198(.a(new_n293), .b(new_n288), .c(new_n287), .d(new_n290), .o1(\s[28] ));
  and002aa1n02x5               g199(.a(new_n288), .b(new_n283), .o(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n286), .c(new_n194), .d(new_n278), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n295), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .c(new_n291), .carry(new_n298));
  aoai13aa1n02x7               g203(.a(new_n298), .b(new_n297), .c(new_n282), .d(new_n279), .o1(new_n299));
  xorc02aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .out0(new_n300));
  norb02aa1n02x5               g205(.a(new_n298), .b(new_n300), .out0(new_n301));
  aoi022aa1n03x5               g206(.a(new_n299), .b(new_n300), .c(new_n296), .d(new_n301), .o1(\s[29] ));
  xorb03aa1n02x5               g207(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g208(.a(new_n292), .b(new_n288), .c(new_n300), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n286), .c(new_n194), .d(new_n278), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n304), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .carry(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n306), .c(new_n282), .d(new_n279), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .out0(new_n309));
  norb02aa1n02x5               g214(.a(new_n307), .b(new_n309), .out0(new_n310));
  aoi022aa1n03x5               g215(.a(new_n308), .b(new_n309), .c(new_n305), .d(new_n310), .o1(\s[30] ));
  nano32aa1n06x5               g216(.a(new_n292), .b(new_n309), .c(new_n288), .d(new_n300), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n286), .c(new_n194), .d(new_n278), .o1(new_n313));
  xorc02aa1n02x5               g218(.a(\a[31] ), .b(\b[30] ), .out0(new_n314));
  and002aa1n02x5               g219(.a(\b[29] ), .b(\a[30] ), .o(new_n315));
  oabi12aa1n02x5               g220(.a(new_n314), .b(\a[30] ), .c(\b[29] ), .out0(new_n316));
  oab012aa1n02x4               g221(.a(new_n316), .b(new_n307), .c(new_n315), .out0(new_n317));
  inv000aa1d42x5               g222(.a(new_n312), .o1(new_n318));
  oao003aa1n02x5               g223(.a(\a[30] ), .b(\b[29] ), .c(new_n307), .carry(new_n319));
  aoai13aa1n02x7               g224(.a(new_n319), .b(new_n318), .c(new_n282), .d(new_n279), .o1(new_n320));
  aoi022aa1n03x5               g225(.a(new_n320), .b(new_n314), .c(new_n313), .d(new_n317), .o1(\s[31] ));
  oaoi03aa1n02x5               g226(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  inv000aa1d42x5               g228(.a(new_n106), .o1(new_n324));
  aoi122aa1n02x5               g229(.a(new_n107), .b(new_n105), .c(new_n324), .d(new_n104), .e(new_n108), .o1(new_n325));
  aoi012aa1n02x5               g230(.a(new_n325), .b(new_n111), .c(new_n324), .o1(\s[4] ));
  xnrb03aa1n02x5               g231(.a(new_n128), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g232(.a(\a[5] ), .b(\b[4] ), .c(new_n128), .o1(new_n328));
  nanb02aa1n02x5               g233(.a(new_n116), .b(new_n111), .out0(new_n329));
  nona22aa1n03x5               g234(.a(new_n329), .b(new_n121), .c(new_n119), .out0(new_n330));
  oaib12aa1n02x5               g235(.a(new_n330), .b(new_n117), .c(new_n328), .out0(\s[6] ));
  aoi022aa1n02x5               g236(.a(new_n330), .b(new_n120), .c(new_n132), .d(new_n114), .o1(new_n332));
  nona23aa1n06x5               g237(.a(new_n330), .b(new_n114), .c(new_n113), .d(new_n119), .out0(new_n333));
  norb02aa1n02x5               g238(.a(new_n333), .b(new_n332), .out0(\s[7] ));
  xnbna2aa1n03x5               g239(.a(new_n112), .b(new_n333), .c(new_n132), .out0(\s[8] ));
  xorb03aa1n02x5               g240(.a(new_n136), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


