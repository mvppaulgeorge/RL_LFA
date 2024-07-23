// Benchmark "adder" written by ABC on Thu Jul 18 08:07:07 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n323, new_n326,
    new_n328, new_n329, new_n330, new_n332, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1n06x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nand02aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1d18x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor002aa1n06x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  tech160nm_finand02aa1n03p5x5 g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor042aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n03x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  tech160nm_fiaoi012aa1n03p5x5 g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n06x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nor022aa1n08x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor002aa1d24x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  tech160nm_finand02aa1n03p5x5 g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n09x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  nor042aa1n03x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand42aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n03x5               g021(.a(new_n112), .b(new_n115), .c(new_n116), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  oai012aa1n02x5               g023(.a(new_n114), .b(new_n118), .c(new_n113), .o1(new_n119));
  aoi012aa1n02x5               g024(.a(new_n108), .b(new_n110), .c(new_n109), .o1(new_n120));
  oai012aa1n06x5               g025(.a(new_n120), .b(new_n112), .c(new_n119), .o1(new_n121));
  aoi012aa1n12x5               g026(.a(new_n121), .b(new_n107), .c(new_n117), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g029(.a(\b[10] ), .b(\a[11] ), .o1(new_n125));
  nand22aa1n04x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  nanb02aa1n02x5               g031(.a(new_n125), .b(new_n126), .out0(new_n127));
  inv000aa1d42x5               g032(.a(\a[10] ), .o1(new_n128));
  inv000aa1d42x5               g033(.a(\b[8] ), .o1(new_n129));
  xroi22aa1d06x4               g034(.a(new_n128), .b(\b[9] ), .c(new_n129), .d(\a[9] ), .out0(new_n130));
  aoai13aa1n06x5               g035(.a(new_n130), .b(new_n121), .c(new_n107), .d(new_n117), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  oai022aa1d18x5               g037(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  nanp02aa1n04x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  xobna2aa1n03x5               g039(.a(new_n127), .b(new_n131), .c(new_n134), .out0(\s[11] ));
  inv000aa1d42x5               g040(.a(new_n125), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n127), .c(new_n131), .d(new_n134), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1d32x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand22aa1n09x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nona23aa1d18x5               g045(.a(new_n140), .b(new_n126), .c(new_n125), .d(new_n139), .out0(new_n141));
  aoi012aa1n12x5               g046(.a(new_n139), .b(new_n125), .c(new_n140), .o1(new_n142));
  oai012aa1d24x5               g047(.a(new_n142), .b(new_n141), .c(new_n134), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  inv000aa1n02x5               g049(.a(new_n100), .o1(new_n145));
  nano23aa1n03x7               g050(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n146));
  aobi12aa1n06x5               g051(.a(new_n106), .b(new_n146), .c(new_n145), .out0(new_n147));
  nano23aa1n02x4               g052(.a(new_n108), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n148));
  nona22aa1n02x4               g053(.a(new_n148), .b(new_n116), .c(new_n115), .out0(new_n149));
  oabi12aa1n18x5               g054(.a(new_n121), .b(new_n147), .c(new_n149), .out0(new_n150));
  inv040aa1n06x5               g055(.a(new_n141), .o1(new_n151));
  nanp03aa1n03x5               g056(.a(new_n150), .b(new_n130), .c(new_n151), .o1(new_n152));
  nor042aa1n06x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n152), .c(new_n144), .out0(\s[13] ));
  inv040aa1n02x5               g061(.a(new_n153), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n155), .o1(new_n158));
  aoai13aa1n03x5               g063(.a(new_n157), .b(new_n158), .c(new_n152), .d(new_n144), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n03x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nano23aa1d12x5               g067(.a(new_n153), .b(new_n161), .c(new_n162), .d(new_n154), .out0(new_n163));
  inv000aa1n02x5               g068(.a(new_n163), .o1(new_n164));
  oaoi03aa1n12x5               g069(.a(\a[14] ), .b(\b[13] ), .c(new_n157), .o1(new_n165));
  aoi012aa1n02x5               g070(.a(new_n165), .b(new_n143), .c(new_n163), .o1(new_n166));
  oai013aa1n03x5               g071(.a(new_n166), .b(new_n131), .c(new_n141), .d(new_n164), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand22aa1n12x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nor042aa1n06x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nand02aa1d06x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aoi112aa1n02x5               g078(.a(new_n169), .b(new_n173), .c(new_n167), .d(new_n170), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n173), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n175));
  norb02aa1n02x7               g080(.a(new_n175), .b(new_n174), .out0(\s[16] ));
  nona23aa1n02x5               g081(.a(new_n172), .b(new_n170), .c(new_n169), .d(new_n171), .out0(new_n177));
  nona23aa1n03x5               g082(.a(new_n130), .b(new_n163), .c(new_n177), .d(new_n141), .out0(new_n178));
  nano23aa1d18x5               g083(.a(new_n169), .b(new_n171), .c(new_n172), .d(new_n170), .out0(new_n179));
  tech160nm_fiao0012aa1n02p5x5 g084(.a(new_n171), .b(new_n169), .c(new_n172), .o(new_n180));
  aoi012aa1n06x5               g085(.a(new_n180), .b(new_n179), .c(new_n165), .o1(new_n181));
  inv020aa1n03x5               g086(.a(new_n181), .o1(new_n182));
  aoi013aa1n09x5               g087(.a(new_n182), .b(new_n143), .c(new_n163), .d(new_n179), .o1(new_n183));
  oai012aa1n12x5               g088(.a(new_n183), .b(new_n122), .c(new_n178), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  nanb02aa1n02x5               g091(.a(\b[16] ), .b(new_n186), .out0(new_n187));
  nand02aa1n02x5               g092(.a(new_n179), .b(new_n163), .o1(new_n188));
  nano22aa1n03x7               g093(.a(new_n188), .b(new_n130), .c(new_n151), .out0(new_n189));
  nona22aa1n02x4               g094(.a(new_n143), .b(new_n164), .c(new_n177), .out0(new_n190));
  nanp02aa1n03x5               g095(.a(new_n190), .b(new_n181), .o1(new_n191));
  xorc02aa1n06x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n191), .c(new_n150), .d(new_n189), .o1(new_n193));
  xnrc02aa1n02x5               g098(.a(\b[17] ), .b(\a[18] ), .out0(new_n194));
  xobna2aa1n03x5               g099(.a(new_n194), .b(new_n193), .c(new_n187), .out0(\s[18] ));
  aoai13aa1n06x5               g100(.a(new_n189), .b(new_n121), .c(new_n107), .d(new_n117), .o1(new_n196));
  inv040aa1d32x5               g101(.a(\a[18] ), .o1(new_n197));
  xroi22aa1d06x4               g102(.a(new_n186), .b(\b[16] ), .c(new_n197), .d(\b[17] ), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  oai022aa1n12x5               g104(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n200));
  oaib12aa1n18x5               g105(.a(new_n200), .b(new_n197), .c(\b[17] ), .out0(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n199), .c(new_n196), .d(new_n183), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nanp02aa1n12x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nanb02aa1n09x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  nor022aa1n16x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanp02aa1n12x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  inv000aa1n02x5               g116(.a(new_n211), .o1(new_n212));
  aoi112aa1n02x5               g117(.a(new_n205), .b(new_n212), .c(new_n202), .d(new_n208), .o1(new_n213));
  inv000aa1n02x5               g118(.a(new_n205), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n201), .o1(new_n215));
  aoai13aa1n03x5               g120(.a(new_n208), .b(new_n215), .c(new_n184), .d(new_n198), .o1(new_n216));
  tech160nm_fiaoi012aa1n02p5x5 g121(.a(new_n211), .b(new_n216), .c(new_n214), .o1(new_n217));
  nor002aa1n02x5               g122(.a(new_n217), .b(new_n213), .o1(\s[20] ));
  nona23aa1d18x5               g123(.a(new_n212), .b(new_n192), .c(new_n194), .d(new_n207), .out0(new_n219));
  nona23aa1d24x5               g124(.a(new_n210), .b(new_n206), .c(new_n205), .d(new_n209), .out0(new_n220));
  oaoi03aa1n02x5               g125(.a(\a[20] ), .b(\b[19] ), .c(new_n214), .o1(new_n221));
  oabi12aa1n18x5               g126(.a(new_n221), .b(new_n220), .c(new_n201), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n04x5               g128(.a(new_n223), .b(new_n219), .c(new_n196), .d(new_n183), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[20] ), .b(\a[21] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[21] ), .b(\a[22] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n226), .b(new_n230), .c(new_n224), .d(new_n228), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n226), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n219), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n228), .b(new_n222), .c(new_n184), .d(new_n233), .o1(new_n234));
  tech160nm_fiaoi012aa1n02p5x5 g139(.a(new_n229), .b(new_n234), .c(new_n232), .o1(new_n235));
  nor002aa1n02x5               g140(.a(new_n235), .b(new_n231), .o1(\s[22] ));
  inv000aa1d42x5               g141(.a(new_n220), .o1(new_n237));
  norp02aa1n09x5               g142(.a(new_n229), .b(new_n227), .o1(new_n238));
  nand23aa1d12x5               g143(.a(new_n198), .b(new_n237), .c(new_n238), .o1(new_n239));
  norp02aa1n02x5               g144(.a(\b[21] ), .b(\a[22] ), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(\b[21] ), .b(\a[22] ), .o1(new_n241));
  tech160nm_fioai012aa1n05x5   g146(.a(new_n241), .b(new_n240), .c(new_n226), .o1(new_n242));
  aobi12aa1n12x5               g147(.a(new_n242), .b(new_n222), .c(new_n238), .out0(new_n243));
  aoai13aa1n04x5               g148(.a(new_n243), .b(new_n239), .c(new_n196), .d(new_n183), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1d32x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(new_n248));
  norp02aa1n04x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  nanp02aa1n02x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n249), .out0(new_n251));
  aoi112aa1n03x4               g156(.a(new_n246), .b(new_n251), .c(new_n244), .d(new_n248), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n246), .o1(new_n253));
  inv040aa1n02x5               g158(.a(new_n239), .o1(new_n254));
  inv000aa1n02x5               g159(.a(new_n243), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n248), .b(new_n255), .c(new_n184), .d(new_n254), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n251), .o1(new_n257));
  aoi012aa1n06x5               g162(.a(new_n257), .b(new_n256), .c(new_n253), .o1(new_n258));
  nor042aa1n03x5               g163(.a(new_n258), .b(new_n252), .o1(\s[24] ));
  nona23aa1n06x5               g164(.a(new_n250), .b(new_n247), .c(new_n246), .d(new_n249), .out0(new_n260));
  inv000aa1n02x5               g165(.a(new_n260), .o1(new_n261));
  nano22aa1n03x7               g166(.a(new_n219), .b(new_n238), .c(new_n261), .out0(new_n262));
  inv000aa1n02x5               g167(.a(new_n262), .o1(new_n263));
  oaoi03aa1n02x5               g168(.a(\a[24] ), .b(\b[23] ), .c(new_n253), .o1(new_n264));
  oab012aa1n06x5               g169(.a(new_n264), .b(new_n260), .c(new_n242), .out0(new_n265));
  nona32aa1n09x5               g170(.a(new_n222), .b(new_n260), .c(new_n229), .d(new_n227), .out0(new_n266));
  nand02aa1d04x5               g171(.a(new_n266), .b(new_n265), .o1(new_n267));
  inv000aa1n03x5               g172(.a(new_n267), .o1(new_n268));
  aoai13aa1n04x5               g173(.a(new_n268), .b(new_n263), .c(new_n196), .d(new_n183), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g175(.a(\b[24] ), .b(\a[25] ), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .out0(new_n272));
  xorc02aa1n12x5               g177(.a(\a[26] ), .b(\b[25] ), .out0(new_n273));
  aoi112aa1n03x4               g178(.a(new_n271), .b(new_n273), .c(new_n269), .d(new_n272), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n271), .o1(new_n275));
  aoai13aa1n03x5               g180(.a(new_n272), .b(new_n267), .c(new_n184), .d(new_n262), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n273), .o1(new_n277));
  tech160nm_fiaoi012aa1n02p5x5 g182(.a(new_n277), .b(new_n276), .c(new_n275), .o1(new_n278));
  nor002aa1n02x5               g183(.a(new_n278), .b(new_n274), .o1(\s[26] ));
  and002aa1n02x5               g184(.a(new_n273), .b(new_n272), .o(new_n280));
  nano22aa1d15x5               g185(.a(new_n239), .b(new_n280), .c(new_n261), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n191), .c(new_n150), .d(new_n189), .o1(new_n282));
  oao003aa1n02x5               g187(.a(\a[26] ), .b(\b[25] ), .c(new_n275), .carry(new_n283));
  aobi12aa1n09x5               g188(.a(new_n283), .b(new_n267), .c(new_n280), .out0(new_n284));
  xorc02aa1n12x5               g189(.a(\a[27] ), .b(\b[26] ), .out0(new_n285));
  xnbna2aa1n03x5               g190(.a(new_n285), .b(new_n282), .c(new_n284), .out0(\s[27] ));
  norp02aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  inv040aa1n03x5               g192(.a(new_n287), .o1(new_n288));
  aobi12aa1n02x7               g193(.a(new_n285), .b(new_n282), .c(new_n284), .out0(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[27] ), .b(\a[28] ), .out0(new_n290));
  nano22aa1n03x5               g195(.a(new_n289), .b(new_n288), .c(new_n290), .out0(new_n291));
  inv000aa1n02x5               g196(.a(new_n280), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n283), .b(new_n292), .c(new_n266), .d(new_n265), .o1(new_n293));
  aoai13aa1n03x5               g198(.a(new_n285), .b(new_n293), .c(new_n184), .d(new_n281), .o1(new_n294));
  tech160nm_fiaoi012aa1n02p5x5 g199(.a(new_n290), .b(new_n294), .c(new_n288), .o1(new_n295));
  norp02aa1n03x5               g200(.a(new_n295), .b(new_n291), .o1(\s[28] ));
  norb02aa1n02x5               g201(.a(new_n285), .b(new_n290), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n293), .c(new_n184), .d(new_n281), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[28] ), .b(\a[29] ), .out0(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n297), .b(new_n282), .c(new_n284), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[29] ));
  xorb03aa1n02x5               g209(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g210(.a(new_n285), .b(new_n300), .c(new_n290), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n293), .c(new_n184), .d(new_n281), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[29] ), .b(\b[28] ), .c(new_n299), .carry(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[29] ), .b(\a[30] ), .out0(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n309), .b(new_n307), .c(new_n308), .o1(new_n310));
  aobi12aa1n02x7               g215(.a(new_n306), .b(new_n282), .c(new_n284), .out0(new_n311));
  nano22aa1n03x5               g216(.a(new_n311), .b(new_n308), .c(new_n309), .out0(new_n312));
  norp02aa1n03x5               g217(.a(new_n310), .b(new_n312), .o1(\s[30] ));
  norb02aa1n02x5               g218(.a(new_n306), .b(new_n309), .out0(new_n314));
  aobi12aa1n02x7               g219(.a(new_n314), .b(new_n282), .c(new_n284), .out0(new_n315));
  oao003aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .c(new_n308), .carry(new_n316));
  xnrc02aa1n02x5               g221(.a(\b[30] ), .b(\a[31] ), .out0(new_n317));
  nano22aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n317), .out0(new_n318));
  aoai13aa1n03x5               g223(.a(new_n314), .b(new_n293), .c(new_n184), .d(new_n281), .o1(new_n319));
  tech160nm_fiaoi012aa1n02p5x5 g224(.a(new_n317), .b(new_n319), .c(new_n316), .o1(new_n320));
  norp02aa1n03x5               g225(.a(new_n320), .b(new_n318), .o1(\s[31] ));
  xnrb03aa1n02x5               g226(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g227(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g229(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n03x5               g230(.a(\a[5] ), .b(\b[4] ), .c(new_n147), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g232(.a(new_n111), .b(new_n110), .out0(new_n328));
  aoai13aa1n02x5               g233(.a(new_n328), .b(new_n113), .c(new_n326), .d(new_n114), .o1(new_n329));
  aoi112aa1n02x5               g234(.a(new_n328), .b(new_n113), .c(new_n326), .d(new_n114), .o1(new_n330));
  norb02aa1n02x5               g235(.a(new_n329), .b(new_n330), .out0(\s[7] ));
  norb02aa1n02x5               g236(.a(new_n109), .b(new_n108), .out0(new_n332));
  inv000aa1d42x5               g237(.a(new_n110), .o1(new_n333));
  xnbna2aa1n03x5               g238(.a(new_n332), .b(new_n329), .c(new_n333), .out0(\s[8] ));
  xorb03aa1n02x5               g239(.a(new_n122), .b(new_n129), .c(\a[9] ), .out0(\s[9] ));
endmodule


