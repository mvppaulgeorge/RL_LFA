// Benchmark "adder" written by ABC on Wed Jul 17 23:08:31 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n320, new_n321, new_n323, new_n326,
    new_n327, new_n329, new_n331;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  nor042aa1d18x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv040aa1n02x5               g003(.a(new_n98), .o1(new_n99));
  nor002aa1d32x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nand02aa1n04x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  norp02aa1n12x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nand02aa1n04x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nona23aa1n09x5               g008(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\b[5] ), .o1(new_n105));
  nanb02aa1d24x5               g010(.a(\a[6] ), .b(new_n105), .out0(new_n106));
  nand42aa1n08x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  oai112aa1n06x5               g012(.a(new_n106), .b(new_n107), .c(\b[4] ), .d(\a[5] ), .o1(new_n108));
  aoi112aa1n06x5               g013(.a(new_n104), .b(new_n108), .c(\a[5] ), .d(\b[4] ), .o1(new_n109));
  and002aa1n02x5               g014(.a(\b[3] ), .b(\a[4] ), .o(new_n110));
  nand22aa1n06x5               g015(.a(\b[0] ), .b(\a[1] ), .o1(new_n111));
  nand02aa1d10x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  nor002aa1n03x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  norb03aa1n03x5               g018(.a(new_n112), .b(new_n111), .c(new_n113), .out0(new_n114));
  nor022aa1n16x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nand42aa1n04x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nanb03aa1n12x5               g021(.a(new_n115), .b(new_n116), .c(new_n112), .out0(new_n117));
  oab012aa1n02x5               g022(.a(new_n115), .b(\a[4] ), .c(\b[3] ), .out0(new_n118));
  oaoi13aa1n06x5               g023(.a(new_n110), .b(new_n118), .c(new_n114), .d(new_n117), .o1(new_n119));
  aoi112aa1n06x5               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  nano23aa1n06x5               g025(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n121));
  nanp03aa1n03x5               g026(.a(new_n121), .b(new_n107), .c(new_n108), .o1(new_n122));
  nona22aa1n03x5               g027(.a(new_n122), .b(new_n120), .c(new_n100), .out0(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n123), .c(new_n109), .d(new_n119), .o1(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n97), .b(new_n125), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g031(.a(\a[10] ), .o1(new_n127));
  inv000aa1d42x5               g032(.a(\b[9] ), .o1(new_n128));
  nand23aa1n03x5               g033(.a(new_n125), .b(new_n97), .c(new_n99), .o1(new_n129));
  nor002aa1d32x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nand42aa1n10x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n06x4               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  oaoi13aa1n02x5               g037(.a(new_n132), .b(new_n129), .c(new_n127), .d(new_n128), .o1(new_n133));
  oai112aa1n03x5               g038(.a(new_n129), .b(new_n132), .c(new_n128), .d(new_n127), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(\s[11] ));
  inv040aa1n06x5               g040(.a(new_n130), .o1(new_n136));
  nor022aa1n16x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand22aa1n09x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n03x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n139), .b(new_n134), .c(new_n136), .out0(\s[12] ));
  inv000aa1n02x5               g045(.a(new_n110), .o1(new_n141));
  tech160nm_fioai012aa1n05x5   g046(.a(new_n118), .b(new_n114), .c(new_n117), .o1(new_n142));
  nand23aa1n06x5               g047(.a(new_n109), .b(new_n142), .c(new_n141), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n100), .o1(new_n144));
  inv000aa1n02x5               g049(.a(new_n120), .o1(new_n145));
  nano22aa1n03x5               g050(.a(new_n104), .b(new_n108), .c(new_n107), .out0(new_n146));
  nano22aa1n03x7               g051(.a(new_n146), .b(new_n144), .c(new_n145), .out0(new_n147));
  nand02aa1d06x5               g052(.a(new_n143), .b(new_n147), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n124), .o1(new_n149));
  nanp03aa1d12x5               g054(.a(new_n97), .b(new_n132), .c(new_n139), .o1(new_n150));
  nona22aa1n09x5               g055(.a(new_n148), .b(new_n149), .c(new_n150), .out0(new_n151));
  nona23aa1n09x5               g056(.a(new_n138), .b(new_n131), .c(new_n130), .d(new_n137), .out0(new_n152));
  tech160nm_fioaoi03aa1n02p5x5 g057(.a(new_n127), .b(new_n128), .c(new_n98), .o1(new_n153));
  oaoi03aa1n09x5               g058(.a(\a[12] ), .b(\b[11] ), .c(new_n136), .o1(new_n154));
  oabi12aa1n09x5               g059(.a(new_n154), .b(new_n152), .c(new_n153), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  xnrc02aa1n12x5               g061(.a(\b[12] ), .b(\a[13] ), .out0(new_n157));
  xobna2aa1n03x5               g062(.a(new_n157), .b(new_n151), .c(new_n156), .out0(\s[13] ));
  tech160nm_fiao0012aa1n02p5x5 g063(.a(new_n157), .b(new_n151), .c(new_n156), .o(new_n159));
  inv040aa1d32x5               g064(.a(\a[14] ), .o1(new_n160));
  inv000aa1d42x5               g065(.a(\b[13] ), .o1(new_n161));
  nanp02aa1n04x5               g066(.a(new_n161), .b(new_n160), .o1(new_n162));
  nand42aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nano22aa1d15x5               g068(.a(new_n157), .b(new_n162), .c(new_n163), .out0(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  nor002aa1n03x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  oaoi03aa1n12x5               g071(.a(new_n160), .b(new_n161), .c(new_n166), .o1(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n165), .c(new_n151), .d(new_n156), .o1(new_n168));
  aoi012aa1n02x5               g073(.a(new_n166), .b(new_n162), .c(new_n163), .o1(new_n169));
  aoi022aa1n02x5               g074(.a(new_n159), .b(new_n169), .c(new_n168), .d(new_n162), .o1(\s[14] ));
  xorb03aa1n03x5               g075(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  tech160nm_fixnrc02aa1n02p5x5 g076(.a(\b[14] ), .b(\a[15] ), .out0(new_n172));
  nanb02aa1n03x5               g077(.a(new_n172), .b(new_n168), .out0(new_n173));
  inv000aa1d42x5               g078(.a(\a[16] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(\b[15] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(new_n175), .b(new_n174), .o1(new_n176));
  norp02aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  xorc02aa1n12x5               g082(.a(\a[16] ), .b(\b[15] ), .out0(new_n178));
  norp02aa1n02x5               g083(.a(new_n178), .b(new_n177), .o1(new_n179));
  nanb02aa1n06x5               g084(.a(new_n172), .b(new_n178), .out0(new_n180));
  xorc02aa1n02x5               g085(.a(\a[14] ), .b(\b[13] ), .out0(new_n181));
  nanb03aa1n09x5               g086(.a(new_n157), .b(new_n181), .c(new_n124), .out0(new_n182));
  nor043aa1n06x5               g087(.a(new_n182), .b(new_n180), .c(new_n150), .o1(new_n183));
  aoai13aa1n06x5               g088(.a(new_n183), .b(new_n123), .c(new_n109), .d(new_n119), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n167), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n178), .b(new_n172), .out0(new_n186));
  aoai13aa1n04x5               g091(.a(new_n186), .b(new_n185), .c(new_n155), .d(new_n164), .o1(new_n187));
  oaoi03aa1n12x5               g092(.a(new_n174), .b(new_n175), .c(new_n177), .o1(new_n188));
  nanp03aa1n06x5               g093(.a(new_n184), .b(new_n187), .c(new_n188), .o1(new_n189));
  aoi022aa1n02x5               g094(.a(new_n173), .b(new_n179), .c(new_n176), .d(new_n189), .o1(\s[16] ));
  xorb03aa1n02x5               g095(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor002aa1d32x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  nona23aa1n03x5               g097(.a(new_n164), .b(new_n186), .c(new_n150), .d(new_n149), .out0(new_n193));
  aoi012aa1n06x5               g098(.a(new_n193), .b(new_n143), .c(new_n147), .o1(new_n194));
  nano23aa1n03x7               g099(.a(new_n130), .b(new_n137), .c(new_n138), .d(new_n131), .out0(new_n195));
  oaoi03aa1n03x5               g100(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n196));
  aoai13aa1n06x5               g101(.a(new_n164), .b(new_n154), .c(new_n195), .d(new_n196), .o1(new_n197));
  aoai13aa1n12x5               g102(.a(new_n188), .b(new_n180), .c(new_n197), .d(new_n167), .o1(new_n198));
  tech160nm_fixorc02aa1n05x5   g103(.a(\a[17] ), .b(\b[16] ), .out0(new_n199));
  nand42aa1d28x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nano22aa1n03x7               g105(.a(new_n192), .b(new_n199), .c(new_n200), .out0(new_n201));
  inv040aa1d30x5               g106(.a(\a[17] ), .o1(new_n202));
  inv040aa1d32x5               g107(.a(\b[16] ), .o1(new_n203));
  aoai13aa1n12x5               g108(.a(new_n200), .b(new_n192), .c(new_n202), .d(new_n203), .o1(new_n204));
  inv040aa1n08x5               g109(.a(new_n204), .o1(new_n205));
  oaoi13aa1n06x5               g110(.a(new_n205), .b(new_n201), .c(new_n194), .d(new_n198), .o1(new_n206));
  obai22aa1n02x7               g111(.a(new_n200), .b(new_n192), .c(\a[17] ), .d(\b[16] ), .out0(new_n207));
  oaoi13aa1n02x5               g112(.a(new_n207), .b(new_n199), .c(new_n194), .d(new_n198), .o1(new_n208));
  oab012aa1n02x4               g113(.a(new_n208), .b(new_n206), .c(new_n192), .out0(\s[18] ));
  aoai13aa1n06x5               g114(.a(new_n201), .b(new_n198), .c(new_n148), .d(new_n183), .o1(new_n210));
  nor002aa1d32x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nanp02aa1n12x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanb02aa1n09x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  xobna2aa1n03x5               g118(.a(new_n213), .b(new_n210), .c(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g120(.a(new_n211), .o1(new_n216));
  aoi012aa1n02x5               g121(.a(new_n213), .b(new_n210), .c(new_n204), .o1(new_n217));
  norp02aa1n12x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand22aa1n12x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nanb02aa1n06x5               g124(.a(new_n218), .b(new_n219), .out0(new_n220));
  nano22aa1n03x5               g125(.a(new_n217), .b(new_n216), .c(new_n220), .out0(new_n221));
  oaoi13aa1n03x5               g126(.a(new_n220), .b(new_n216), .c(new_n206), .d(new_n213), .o1(new_n222));
  nor002aa1n02x5               g127(.a(new_n222), .b(new_n221), .o1(\s[20] ));
  nano23aa1n06x5               g128(.a(new_n211), .b(new_n218), .c(new_n219), .d(new_n212), .out0(new_n224));
  nand02aa1d04x5               g129(.a(new_n201), .b(new_n224), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n198), .c(new_n148), .d(new_n183), .o1(new_n227));
  tech160nm_fiaoi012aa1n04x5   g132(.a(new_n218), .b(new_n211), .c(new_n219), .o1(new_n228));
  oai013aa1d12x5               g133(.a(new_n228), .b(new_n204), .c(new_n213), .d(new_n220), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  xnrc02aa1n12x5               g135(.a(\b[20] ), .b(\a[21] ), .out0(new_n231));
  xobna2aa1n03x5               g136(.a(new_n231), .b(new_n227), .c(new_n230), .out0(\s[21] ));
  nor042aa1n04x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  inv040aa1n03x5               g138(.a(new_n233), .o1(new_n234));
  tech160nm_fiaoi012aa1n02p5x5 g139(.a(new_n231), .b(new_n227), .c(new_n230), .o1(new_n235));
  xnrc02aa1n06x5               g140(.a(\b[21] ), .b(\a[22] ), .out0(new_n236));
  nano22aa1n03x7               g141(.a(new_n235), .b(new_n234), .c(new_n236), .out0(new_n237));
  oaoi13aa1n03x5               g142(.a(new_n229), .b(new_n226), .c(new_n194), .d(new_n198), .o1(new_n238));
  oaoi13aa1n03x5               g143(.a(new_n236), .b(new_n234), .c(new_n238), .d(new_n231), .o1(new_n239));
  nor002aa1n02x5               g144(.a(new_n239), .b(new_n237), .o1(\s[22] ));
  nor042aa1n03x5               g145(.a(new_n236), .b(new_n231), .o1(new_n241));
  and003aa1n02x5               g146(.a(new_n201), .b(new_n241), .c(new_n224), .o(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n198), .c(new_n148), .d(new_n183), .o1(new_n243));
  oao003aa1n12x5               g148(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .carry(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n229), .c(new_n241), .o1(new_n246));
  xnrc02aa1n12x5               g151(.a(\b[22] ), .b(\a[23] ), .out0(new_n247));
  xobna2aa1n03x5               g152(.a(new_n247), .b(new_n243), .c(new_n246), .out0(\s[23] ));
  nor042aa1n06x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  aoi012aa1n03x5               g155(.a(new_n247), .b(new_n243), .c(new_n246), .o1(new_n251));
  xnrc02aa1n12x5               g156(.a(\b[23] ), .b(\a[24] ), .out0(new_n252));
  nano22aa1n02x5               g157(.a(new_n251), .b(new_n250), .c(new_n252), .out0(new_n253));
  inv000aa1n03x5               g158(.a(new_n246), .o1(new_n254));
  oaoi13aa1n03x5               g159(.a(new_n254), .b(new_n242), .c(new_n194), .d(new_n198), .o1(new_n255));
  oaoi13aa1n03x5               g160(.a(new_n252), .b(new_n250), .c(new_n255), .d(new_n247), .o1(new_n256));
  norp02aa1n03x5               g161(.a(new_n256), .b(new_n253), .o1(\s[24] ));
  nor042aa1n02x5               g162(.a(new_n252), .b(new_n247), .o1(new_n258));
  nano22aa1n02x4               g163(.a(new_n225), .b(new_n241), .c(new_n258), .out0(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n198), .c(new_n148), .d(new_n183), .o1(new_n260));
  inv000aa1n02x5               g165(.a(new_n228), .o1(new_n261));
  aoai13aa1n06x5               g166(.a(new_n241), .b(new_n261), .c(new_n224), .d(new_n205), .o1(new_n262));
  inv020aa1n02x5               g167(.a(new_n258), .o1(new_n263));
  oao003aa1n02x5               g168(.a(\a[24] ), .b(\b[23] ), .c(new_n250), .carry(new_n264));
  aoai13aa1n12x5               g169(.a(new_n264), .b(new_n263), .c(new_n262), .d(new_n244), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  xnrc02aa1n12x5               g171(.a(\b[24] ), .b(\a[25] ), .out0(new_n267));
  xobna2aa1n03x5               g172(.a(new_n267), .b(new_n260), .c(new_n266), .out0(\s[25] ));
  nor042aa1n03x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  aoi012aa1n03x5               g175(.a(new_n267), .b(new_n260), .c(new_n266), .o1(new_n271));
  tech160nm_fixnrc02aa1n05x5   g176(.a(\b[25] ), .b(\a[26] ), .out0(new_n272));
  nano22aa1n02x4               g177(.a(new_n271), .b(new_n270), .c(new_n272), .out0(new_n273));
  oaoi13aa1n02x7               g178(.a(new_n265), .b(new_n259), .c(new_n194), .d(new_n198), .o1(new_n274));
  oaoi13aa1n03x5               g179(.a(new_n272), .b(new_n270), .c(new_n274), .d(new_n267), .o1(new_n275));
  nor002aa1n02x5               g180(.a(new_n275), .b(new_n273), .o1(\s[26] ));
  nor042aa1n06x5               g181(.a(new_n272), .b(new_n267), .o1(new_n277));
  nano32aa1n03x7               g182(.a(new_n225), .b(new_n277), .c(new_n241), .d(new_n258), .out0(new_n278));
  aoai13aa1n12x5               g183(.a(new_n278), .b(new_n198), .c(new_n148), .d(new_n183), .o1(new_n279));
  oao003aa1n02x5               g184(.a(\a[26] ), .b(\b[25] ), .c(new_n270), .carry(new_n280));
  aobi12aa1n18x5               g185(.a(new_n280), .b(new_n265), .c(new_n277), .out0(new_n281));
  xorc02aa1n12x5               g186(.a(\a[27] ), .b(\b[26] ), .out0(new_n282));
  xnbna2aa1n03x5               g187(.a(new_n282), .b(new_n279), .c(new_n281), .out0(\s[27] ));
  nor042aa1n03x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n284), .o1(new_n285));
  aobi12aa1n06x5               g190(.a(new_n282), .b(new_n279), .c(new_n281), .out0(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[27] ), .b(\a[28] ), .out0(new_n287));
  nano22aa1n03x7               g192(.a(new_n286), .b(new_n285), .c(new_n287), .out0(new_n288));
  aoai13aa1n03x5               g193(.a(new_n258), .b(new_n245), .c(new_n229), .d(new_n241), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n277), .o1(new_n290));
  aoai13aa1n04x5               g195(.a(new_n280), .b(new_n290), .c(new_n289), .d(new_n264), .o1(new_n291));
  aoai13aa1n03x5               g196(.a(new_n282), .b(new_n291), .c(new_n189), .d(new_n278), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n287), .b(new_n292), .c(new_n285), .o1(new_n293));
  norp02aa1n03x5               g198(.a(new_n293), .b(new_n288), .o1(\s[28] ));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  norb02aa1n02x5               g200(.a(new_n282), .b(new_n287), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n291), .c(new_n189), .d(new_n278), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .c(new_n285), .carry(new_n298));
  tech160nm_fiaoi012aa1n02p5x5 g203(.a(new_n295), .b(new_n297), .c(new_n298), .o1(new_n299));
  aobi12aa1n06x5               g204(.a(new_n296), .b(new_n279), .c(new_n281), .out0(new_n300));
  nano22aa1n03x7               g205(.a(new_n300), .b(new_n295), .c(new_n298), .out0(new_n301));
  norp02aa1n03x5               g206(.a(new_n299), .b(new_n301), .o1(\s[29] ));
  xorb03aa1n02x5               g207(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g208(.a(new_n282), .b(new_n295), .c(new_n287), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n291), .c(new_n189), .d(new_n278), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[29] ), .b(\a[30] ), .out0(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  aobi12aa1n06x5               g213(.a(new_n304), .b(new_n279), .c(new_n281), .out0(new_n309));
  nano22aa1n03x7               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[30] ));
  norb02aa1n02x5               g216(.a(new_n304), .b(new_n307), .out0(new_n312));
  aobi12aa1n06x5               g217(.a(new_n312), .b(new_n279), .c(new_n281), .out0(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n306), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[30] ), .b(\a[31] ), .out0(new_n315));
  nano22aa1n03x7               g220(.a(new_n313), .b(new_n314), .c(new_n315), .out0(new_n316));
  aoai13aa1n02x5               g221(.a(new_n312), .b(new_n291), .c(new_n189), .d(new_n278), .o1(new_n317));
  tech160nm_fiaoi012aa1n02p5x5 g222(.a(new_n315), .b(new_n317), .c(new_n314), .o1(new_n318));
  norp02aa1n03x5               g223(.a(new_n318), .b(new_n316), .o1(\s[31] ));
  norb02aa1n02x5               g224(.a(new_n116), .b(new_n115), .out0(new_n320));
  oaoi13aa1n02x5               g225(.a(new_n320), .b(new_n112), .c(new_n111), .d(new_n113), .o1(new_n321));
  oab012aa1n02x4               g226(.a(new_n321), .b(new_n114), .c(new_n117), .out0(\s[3] ));
  oabi12aa1n02x5               g227(.a(new_n115), .b(new_n114), .c(new_n117), .out0(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g229(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g230(.a(new_n142), .b(new_n141), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[5] ), .b(\b[4] ), .c(new_n326), .carry(new_n327));
  xnbna2aa1n03x5               g232(.a(new_n327), .b(new_n106), .c(new_n107), .out0(\s[6] ));
  tech160nm_fioaoi03aa1n03p5x5 g233(.a(\a[6] ), .b(\b[5] ), .c(new_n327), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  tech160nm_fiaoi012aa1n05x5   g235(.a(new_n102), .b(new_n329), .c(new_n103), .o1(new_n331));
  xnbna2aa1n03x5               g236(.a(new_n331), .b(new_n144), .c(new_n101), .out0(\s[8] ));
  xnbna2aa1n03x5               g237(.a(new_n124), .b(new_n143), .c(new_n147), .out0(\s[9] ));
endmodule


