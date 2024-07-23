// Benchmark "adder" written by ABC on Wed Jul 17 23:28:13 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n321, new_n323, new_n324,
    new_n325, new_n328, new_n330, new_n332, new_n333, new_n334;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor002aa1d32x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(new_n98), .o1(new_n99));
  nor022aa1n08x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nor002aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nand02aa1d04x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nanp02aa1n12x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nona22aa1n09x5               g010(.a(new_n104), .b(new_n105), .c(new_n103), .out0(new_n106));
  nand42aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nano22aa1n02x4               g012(.a(new_n100), .b(new_n104), .c(new_n107), .out0(new_n108));
  aobi12aa1n02x7               g013(.a(new_n102), .b(new_n108), .c(new_n106), .out0(new_n109));
  inv040aa1d32x5               g014(.a(\a[4] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\b[3] ), .o1(new_n111));
  tech160nm_finand02aa1n03p5x5 g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand42aa1d28x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  oai112aa1n06x5               g018(.a(new_n112), .b(new_n113), .c(new_n111), .d(new_n110), .o1(new_n114));
  inv000aa1n02x5               g019(.a(new_n114), .o1(new_n115));
  xorc02aa1n12x5               g020(.a(\a[8] ), .b(\b[7] ), .out0(new_n116));
  oai022aa1d18x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  nor022aa1n16x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  and002aa1n12x5               g023(.a(\b[5] ), .b(\a[6] ), .o(new_n119));
  nor003aa1n02x5               g024(.a(new_n117), .b(new_n119), .c(new_n118), .o1(new_n120));
  nand03aa1n02x5               g025(.a(new_n115), .b(new_n120), .c(new_n116), .o1(new_n121));
  norb03aa1n06x5               g026(.a(new_n113), .b(new_n119), .c(new_n118), .out0(new_n122));
  aob012aa1n02x5               g027(.a(new_n118), .b(\b[7] ), .c(\a[8] ), .out0(new_n123));
  oai012aa1n02x5               g028(.a(new_n123), .b(\b[7] ), .c(\a[8] ), .o1(new_n124));
  aoi013aa1n09x5               g029(.a(new_n124), .b(new_n122), .c(new_n116), .d(new_n117), .o1(new_n125));
  oai012aa1n06x5               g030(.a(new_n125), .b(new_n109), .c(new_n121), .o1(new_n126));
  nand42aa1n03x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nanb02aa1n12x5               g032(.a(new_n98), .b(new_n127), .out0(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(new_n126), .b(new_n129), .o1(new_n130));
  xobna2aa1n03x5               g035(.a(new_n97), .b(new_n130), .c(new_n99), .out0(\s[10] ));
  and002aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o(new_n132));
  aoi112aa1n03x5               g037(.a(new_n97), .b(new_n98), .c(new_n126), .d(new_n129), .o1(new_n133));
  nor022aa1n08x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  inv030aa1n03x5               g041(.a(new_n134), .o1(new_n137));
  aob012aa1n02x5               g042(.a(new_n137), .b(\b[9] ), .c(\a[10] ), .out0(new_n138));
  aoi112aa1n03x5               g043(.a(new_n133), .b(new_n138), .c(\a[11] ), .d(\b[10] ), .o1(new_n139));
  oaoi13aa1n02x5               g044(.a(new_n139), .b(new_n136), .c(new_n133), .d(new_n132), .o1(\s[11] ));
  orn002aa1n24x5               g045(.a(\a[12] ), .b(\b[11] ), .o(new_n141));
  nand02aa1d06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(new_n141), .b(new_n142), .o1(new_n143));
  oai012aa1n02x5               g048(.a(new_n143), .b(new_n139), .c(new_n134), .o1(new_n144));
  nona22aa1n03x5               g049(.a(new_n137), .b(new_n139), .c(new_n143), .out0(new_n145));
  nanp02aa1n03x5               g050(.a(new_n145), .b(new_n144), .o1(\s[12] ));
  norb03aa1n03x5               g051(.a(new_n104), .b(new_n103), .c(new_n105), .out0(new_n147));
  nanb03aa1n03x5               g052(.a(new_n100), .b(new_n107), .c(new_n104), .out0(new_n148));
  tech160nm_fioai012aa1n03p5x5 g053(.a(new_n102), .b(new_n147), .c(new_n148), .o1(new_n149));
  xnrc02aa1n02x5               g054(.a(\b[7] ), .b(\a[8] ), .out0(new_n150));
  norp02aa1n02x5               g055(.a(\b[5] ), .b(\a[6] ), .o1(new_n151));
  norp02aa1n02x5               g056(.a(\b[4] ), .b(\a[5] ), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n118), .b(\a[6] ), .c(\b[5] ), .o1(new_n153));
  nona22aa1n02x4               g058(.a(new_n153), .b(new_n152), .c(new_n151), .out0(new_n154));
  nona32aa1n09x5               g059(.a(new_n149), .b(new_n154), .c(new_n150), .d(new_n114), .out0(new_n155));
  norp02aa1n02x5               g060(.a(new_n97), .b(new_n143), .o1(new_n156));
  nona22aa1n03x5               g061(.a(new_n156), .b(new_n136), .c(new_n128), .out0(new_n157));
  inv000aa1n02x5               g062(.a(new_n142), .o1(new_n158));
  nor002aa1n12x5               g063(.a(\b[9] ), .b(\a[10] ), .o1(new_n159));
  aoi022aa1d24x5               g064(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n160));
  oai012aa1d24x5               g065(.a(new_n160), .b(new_n98), .c(new_n159), .o1(new_n161));
  aoi013aa1n09x5               g066(.a(new_n158), .b(new_n161), .c(new_n137), .d(new_n141), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  aoai13aa1n04x5               g068(.a(new_n163), .b(new_n157), .c(new_n155), .d(new_n125), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n24x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nand22aa1n03x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  aoi012aa1n02x5               g072(.a(new_n166), .b(new_n164), .c(new_n167), .o1(new_n168));
  xnrb03aa1n02x5               g073(.a(new_n168), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n12x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand22aa1n12x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nano23aa1n06x5               g076(.a(new_n166), .b(new_n170), .c(new_n171), .d(new_n167), .out0(new_n172));
  aoi012aa1n02x5               g077(.a(new_n170), .b(new_n166), .c(new_n171), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  nor002aa1n20x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nand42aa1n04x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n174), .c(new_n164), .d(new_n172), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n178), .b(new_n174), .c(new_n164), .d(new_n172), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(\s[15] ));
  inv000aa1d42x5               g086(.a(new_n175), .o1(new_n182));
  nor002aa1n16x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nand22aa1n09x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(new_n183), .b(new_n184), .out0(new_n185));
  aob012aa1n03x5               g090(.a(new_n185), .b(new_n179), .c(new_n182), .out0(new_n186));
  nona22aa1n02x4               g091(.a(new_n179), .b(new_n185), .c(new_n175), .out0(new_n187));
  nanp02aa1n03x5               g092(.a(new_n186), .b(new_n187), .o1(\s[16] ));
  nona23aa1n02x4               g093(.a(new_n135), .b(new_n127), .c(new_n98), .d(new_n134), .out0(new_n189));
  nona23aa1n03x5               g094(.a(new_n171), .b(new_n167), .c(new_n166), .d(new_n170), .out0(new_n190));
  nona23aa1n09x5               g095(.a(new_n184), .b(new_n176), .c(new_n175), .d(new_n183), .out0(new_n191));
  nor042aa1n04x5               g096(.a(new_n191), .b(new_n190), .o1(new_n192));
  nona32aa1n03x5               g097(.a(new_n192), .b(new_n189), .c(new_n143), .d(new_n97), .out0(new_n193));
  inv000aa1d42x5               g098(.a(new_n183), .o1(new_n194));
  inv000aa1d42x5               g099(.a(new_n184), .o1(new_n195));
  aoai13aa1n02x7               g100(.a(new_n176), .b(new_n170), .c(new_n166), .d(new_n171), .o1(new_n196));
  aoai13aa1n06x5               g101(.a(new_n194), .b(new_n195), .c(new_n196), .d(new_n182), .o1(new_n197));
  aoi012aa1n12x5               g102(.a(new_n197), .b(new_n162), .c(new_n192), .o1(new_n198));
  aoai13aa1n12x5               g103(.a(new_n198), .b(new_n193), .c(new_n155), .d(new_n125), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d30x5               g105(.a(\a[18] ), .o1(new_n201));
  inv040aa1d30x5               g106(.a(\a[17] ), .o1(new_n202));
  inv000aa1d42x5               g107(.a(\b[16] ), .o1(new_n203));
  oaoi03aa1n03x5               g108(.a(new_n202), .b(new_n203), .c(new_n199), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[17] ), .c(new_n201), .out0(\s[18] ));
  xroi22aa1d06x4               g110(.a(new_n202), .b(\b[16] ), .c(new_n201), .d(\b[17] ), .out0(new_n206));
  aoi112aa1n09x5               g111(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n207));
  oabi12aa1n02x5               g112(.a(new_n207), .b(\a[18] ), .c(\b[17] ), .out0(new_n208));
  nor042aa1d18x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand42aa1n06x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  norb02aa1n12x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n208), .c(new_n199), .d(new_n206), .o1(new_n212));
  aoi112aa1n02x5               g117(.a(new_n211), .b(new_n208), .c(new_n199), .d(new_n206), .o1(new_n213));
  norb02aa1n02x7               g118(.a(new_n212), .b(new_n213), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g120(.a(new_n209), .o1(new_n216));
  nor002aa1d24x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nand02aa1d28x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  norb02aa1d27x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aob012aa1n03x5               g125(.a(new_n220), .b(new_n212), .c(new_n216), .out0(new_n221));
  nona22aa1n02x4               g126(.a(new_n212), .b(new_n220), .c(new_n209), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n221), .b(new_n222), .o1(\s[20] ));
  nano23aa1n03x5               g128(.a(new_n209), .b(new_n217), .c(new_n218), .d(new_n210), .out0(new_n224));
  nand02aa1d04x5               g129(.a(new_n206), .b(new_n224), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoi012aa1d24x5               g131(.a(new_n217), .b(new_n209), .c(new_n218), .o1(new_n227));
  nor042aa1n02x5               g132(.a(\b[17] ), .b(\a[18] ), .o1(new_n228));
  oai112aa1n06x5               g133(.a(new_n211), .b(new_n219), .c(new_n207), .d(new_n228), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(new_n229), .b(new_n227), .o1(new_n230));
  xorc02aa1n06x5               g135(.a(\a[21] ), .b(\b[20] ), .out0(new_n231));
  aoai13aa1n06x5               g136(.a(new_n231), .b(new_n230), .c(new_n199), .d(new_n226), .o1(new_n232));
  aoi112aa1n02x5               g137(.a(new_n231), .b(new_n230), .c(new_n199), .d(new_n226), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n232), .b(new_n233), .out0(\s[21] ));
  nor042aa1d18x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n235), .o1(new_n236));
  xnrc02aa1n12x5               g141(.a(\b[21] ), .b(\a[22] ), .out0(new_n237));
  aob012aa1n03x5               g142(.a(new_n237), .b(new_n232), .c(new_n236), .out0(new_n238));
  nona22aa1n02x4               g143(.a(new_n232), .b(new_n237), .c(new_n235), .out0(new_n239));
  nanp02aa1n03x5               g144(.a(new_n238), .b(new_n239), .o1(\s[22] ));
  norb02aa1n03x5               g145(.a(new_n192), .b(new_n157), .out0(new_n241));
  nanp03aa1n02x5               g146(.a(new_n161), .b(new_n137), .c(new_n141), .o1(new_n242));
  nona23aa1n02x4               g147(.a(new_n242), .b(new_n172), .c(new_n191), .d(new_n158), .out0(new_n243));
  nanb02aa1n03x5               g148(.a(new_n197), .b(new_n243), .out0(new_n244));
  nanb02aa1n12x5               g149(.a(new_n237), .b(new_n231), .out0(new_n245));
  nano22aa1n02x4               g150(.a(new_n245), .b(new_n206), .c(new_n224), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n244), .c(new_n126), .d(new_n241), .o1(new_n247));
  oao003aa1n02x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n236), .carry(new_n248));
  aoai13aa1n12x5               g153(.a(new_n248), .b(new_n245), .c(new_n229), .d(new_n227), .o1(new_n249));
  nanb02aa1n03x5               g154(.a(new_n249), .b(new_n247), .out0(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[23] ), .b(\b[22] ), .out0(new_n253));
  xorc02aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n255), .b(new_n252), .c(new_n250), .d(new_n253), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n253), .b(new_n249), .c(new_n199), .d(new_n246), .o1(new_n257));
  nona22aa1n03x5               g162(.a(new_n257), .b(new_n255), .c(new_n252), .out0(new_n258));
  nanp02aa1n03x5               g163(.a(new_n256), .b(new_n258), .o1(\s[24] ));
  norb02aa1n02x5               g164(.a(new_n231), .b(new_n237), .out0(new_n260));
  inv000aa1d42x5               g165(.a(\a[23] ), .o1(new_n261));
  inv020aa1n04x5               g166(.a(\a[24] ), .o1(new_n262));
  xroi22aa1d06x4               g167(.a(new_n261), .b(\b[22] ), .c(new_n262), .d(\b[23] ), .out0(new_n263));
  nano22aa1n03x7               g168(.a(new_n225), .b(new_n260), .c(new_n263), .out0(new_n264));
  nanb02aa1n02x5               g169(.a(\b[22] ), .b(new_n261), .out0(new_n265));
  oaoi03aa1n12x5               g170(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .o1(new_n266));
  aoi012aa1d18x5               g171(.a(new_n266), .b(new_n249), .c(new_n263), .o1(new_n267));
  aob012aa1n03x5               g172(.a(new_n267), .b(new_n199), .c(new_n264), .out0(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  xorc02aa1n06x5               g175(.a(\a[25] ), .b(\b[24] ), .out0(new_n271));
  xorc02aa1n12x5               g176(.a(\a[26] ), .b(\b[25] ), .out0(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n274));
  inv000aa1n02x5               g179(.a(new_n267), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n271), .b(new_n275), .c(new_n264), .d(new_n199), .o1(new_n276));
  nona22aa1n02x4               g181(.a(new_n276), .b(new_n273), .c(new_n270), .out0(new_n277));
  nanp02aa1n03x5               g182(.a(new_n274), .b(new_n277), .o1(\s[26] ));
  and002aa1n02x5               g183(.a(new_n272), .b(new_n271), .o(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n266), .c(new_n249), .d(new_n263), .o1(new_n280));
  nano32aa1n03x7               g185(.a(new_n225), .b(new_n279), .c(new_n260), .d(new_n263), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n244), .c(new_n126), .d(new_n241), .o1(new_n282));
  nanp02aa1n02x5               g187(.a(\b[25] ), .b(\a[26] ), .o1(new_n283));
  oai022aa1n02x5               g188(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(new_n284), .b(new_n283), .o1(new_n285));
  nand23aa1n06x5               g190(.a(new_n282), .b(new_n280), .c(new_n285), .o1(new_n286));
  xorb03aa1n02x5               g191(.a(new_n286), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  xorc02aa1n12x5               g193(.a(\a[27] ), .b(\b[26] ), .out0(new_n289));
  nor022aa1n06x5               g194(.a(\b[27] ), .b(\a[28] ), .o1(new_n290));
  nand42aa1n03x5               g195(.a(\b[27] ), .b(\a[28] ), .o1(new_n291));
  nanb02aa1n06x5               g196(.a(new_n290), .b(new_n291), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n288), .c(new_n286), .d(new_n289), .o1(new_n293));
  nand42aa1n02x5               g198(.a(new_n286), .b(new_n289), .o1(new_n294));
  nona22aa1n03x5               g199(.a(new_n294), .b(new_n292), .c(new_n288), .out0(new_n295));
  nanp02aa1n03x5               g200(.a(new_n295), .b(new_n293), .o1(\s[28] ));
  norb02aa1n03x5               g201(.a(new_n289), .b(new_n292), .out0(new_n297));
  nanp02aa1n03x5               g202(.a(new_n286), .b(new_n297), .o1(new_n298));
  aoi022aa1n09x5               g203(.a(new_n199), .b(new_n281), .c(new_n283), .d(new_n284), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n297), .o1(new_n300));
  aoi012aa1n02x5               g205(.a(new_n290), .b(new_n288), .c(new_n291), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n300), .c(new_n299), .d(new_n280), .o1(new_n302));
  xorc02aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .out0(new_n303));
  norb02aa1n02x5               g208(.a(new_n301), .b(new_n303), .out0(new_n304));
  aoi022aa1n03x5               g209(.a(new_n302), .b(new_n303), .c(new_n298), .d(new_n304), .o1(\s[29] ));
  xorb03aa1n02x5               g210(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g211(.a(new_n292), .b(new_n303), .c(new_n289), .out0(new_n307));
  nanb02aa1n02x5               g212(.a(new_n307), .b(new_n286), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[29] ), .b(\b[28] ), .c(new_n301), .carry(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n307), .c(new_n299), .d(new_n280), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .out0(new_n311));
  norb02aa1n02x5               g216(.a(new_n309), .b(new_n311), .out0(new_n312));
  aoi022aa1n03x5               g217(.a(new_n310), .b(new_n311), .c(new_n308), .d(new_n312), .o1(\s[30] ));
  nanp03aa1n02x5               g218(.a(new_n297), .b(new_n303), .c(new_n311), .o1(new_n314));
  nanb02aa1n02x5               g219(.a(new_n314), .b(new_n286), .out0(new_n315));
  xorc02aa1n02x5               g220(.a(\a[31] ), .b(\b[30] ), .out0(new_n316));
  oao003aa1n02x5               g221(.a(\a[30] ), .b(\b[29] ), .c(new_n309), .carry(new_n317));
  norb02aa1n02x5               g222(.a(new_n317), .b(new_n316), .out0(new_n318));
  aoai13aa1n03x5               g223(.a(new_n317), .b(new_n314), .c(new_n299), .d(new_n280), .o1(new_n319));
  aoi022aa1n03x5               g224(.a(new_n319), .b(new_n316), .c(new_n315), .d(new_n318), .o1(\s[31] ));
  nanb02aa1n02x5               g225(.a(new_n100), .b(new_n107), .out0(new_n321));
  xnbna2aa1n03x5               g226(.a(new_n321), .b(new_n106), .c(new_n104), .out0(\s[3] ));
  xorc02aa1n02x5               g227(.a(\a[4] ), .b(\b[3] ), .out0(new_n323));
  aoi112aa1n02x5               g228(.a(new_n323), .b(new_n100), .c(new_n108), .d(new_n106), .o1(new_n324));
  oaib12aa1n06x5               g229(.a(new_n149), .b(new_n111), .c(\a[4] ), .out0(new_n325));
  oab012aa1n02x4               g230(.a(new_n324), .b(new_n325), .c(new_n101), .out0(\s[4] ));
  xnrb03aa1n02x5               g231(.a(new_n325), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oao003aa1n03x5               g232(.a(\a[5] ), .b(\b[4] ), .c(new_n325), .carry(new_n328));
  xnrb03aa1n02x5               g233(.a(new_n328), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fioaoi03aa1n03p5x5 g234(.a(\a[6] ), .b(\b[5] ), .c(new_n328), .o1(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoai13aa1n02x5               g236(.a(new_n150), .b(new_n118), .c(new_n330), .d(new_n113), .o1(new_n332));
  aoi012aa1n02x5               g237(.a(new_n118), .b(new_n330), .c(new_n113), .o1(new_n333));
  nanp02aa1n02x5               g238(.a(new_n333), .b(new_n116), .o1(new_n334));
  nanp02aa1n02x5               g239(.a(new_n334), .b(new_n332), .o1(\s[8] ));
  xnbna2aa1n03x5               g240(.a(new_n129), .b(new_n155), .c(new_n125), .out0(\s[9] ));
endmodule


