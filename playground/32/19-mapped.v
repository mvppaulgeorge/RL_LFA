// Benchmark "adder" written by ABC on Thu Jul 18 04:30:35 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n256, new_n257, new_n258,
    new_n259, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n333, new_n334, new_n335, new_n337, new_n338, new_n340,
    new_n341, new_n343, new_n344, new_n346, new_n347, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  orn002aa1n02x5               g002(.a(\a[2] ), .b(\b[1] ), .o(new_n98));
  nanp02aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  aob012aa1n03x5               g004(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(new_n100));
  orn002aa1n24x5               g005(.a(\a[3] ), .b(\b[2] ), .o(new_n101));
  nanp02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanp02aa1n09x5               g007(.a(new_n101), .b(new_n102), .o1(new_n103));
  oa0022aa1n02x5               g008(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n104));
  aoai13aa1n06x5               g009(.a(new_n104), .b(new_n103), .c(new_n100), .d(new_n98), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  norp02aa1n03x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  aoi012aa1n02x7               g012(.a(new_n107), .b(\a[6] ), .c(\b[5] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\a[8] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[7] ), .o1(new_n110));
  aoi022aa1n02x5               g015(.a(new_n110), .b(new_n109), .c(\a[4] ), .d(\b[3] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor022aa1n16x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nona23aa1n03x5               g020(.a(new_n114), .b(new_n112), .c(new_n115), .d(new_n113), .out0(new_n116));
  nano32aa1n03x5               g021(.a(new_n116), .b(new_n111), .c(new_n108), .d(new_n106), .out0(new_n117));
  nanp02aa1n09x5               g022(.a(new_n117), .b(new_n105), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nano22aa1n03x7               g024(.a(new_n113), .b(new_n119), .c(new_n114), .out0(new_n120));
  oaih22aa1n04x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  tech160nm_fixorc02aa1n05x5   g026(.a(\a[8] ), .b(\b[7] ), .out0(new_n122));
  oao003aa1n02x5               g027(.a(new_n109), .b(new_n110), .c(new_n113), .carry(new_n123));
  aoi013aa1n09x5               g028(.a(new_n123), .b(new_n120), .c(new_n122), .d(new_n121), .o1(new_n124));
  nanp02aa1n06x5               g029(.a(new_n118), .b(new_n124), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nor022aa1n06x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nanp02aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n97), .c(new_n125), .d(new_n126), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n126), .b(new_n97), .out0(new_n131));
  aoi112aa1n02x5               g036(.a(new_n129), .b(new_n97), .c(new_n125), .d(new_n131), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n130), .b(new_n132), .out0(\s[10] ));
  oai012aa1n02x5               g038(.a(new_n128), .b(new_n127), .c(new_n97), .o1(new_n134));
  nona23aa1n06x5               g039(.a(new_n128), .b(new_n126), .c(new_n97), .d(new_n127), .out0(new_n135));
  nanb02aa1n02x5               g040(.a(new_n135), .b(new_n125), .out0(new_n136));
  xnrc02aa1n12x5               g041(.a(\b[10] ), .b(\a[11] ), .out0(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n136), .c(new_n134), .out0(\s[11] ));
  aob012aa1n02x5               g044(.a(new_n138), .b(new_n136), .c(new_n134), .out0(new_n140));
  nor042aa1n09x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand22aa1n04x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanb02aa1n03x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  inv000aa1d42x5               g048(.a(\a[11] ), .o1(new_n144));
  inv000aa1d42x5               g049(.a(\b[10] ), .o1(new_n145));
  aboi22aa1n03x5               g050(.a(new_n141), .b(new_n142), .c(new_n144), .d(new_n145), .out0(new_n146));
  nand42aa1n02x5               g051(.a(new_n145), .b(new_n144), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n137), .c(new_n136), .d(new_n134), .o1(new_n148));
  aboi22aa1n03x5               g053(.a(new_n143), .b(new_n148), .c(new_n140), .d(new_n146), .out0(\s[12] ));
  norp03aa1n04x5               g054(.a(new_n135), .b(new_n137), .c(new_n143), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n125), .b(new_n150), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[10] ), .b(\a[11] ), .o1(new_n152));
  nanb03aa1n03x5               g057(.a(new_n141), .b(new_n142), .c(new_n152), .out0(new_n153));
  oai112aa1n04x5               g058(.a(new_n128), .b(new_n147), .c(new_n127), .d(new_n97), .o1(new_n154));
  aoi013aa1n06x4               g059(.a(new_n141), .b(new_n142), .c(new_n144), .d(new_n145), .o1(new_n155));
  tech160nm_fioai012aa1n05x5   g060(.a(new_n155), .b(new_n154), .c(new_n153), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(new_n156), .b(new_n151), .out0(new_n157));
  nor002aa1d32x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nand02aa1n04x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  oai112aa1n02x5               g066(.a(new_n155), .b(new_n160), .c(new_n154), .d(new_n153), .o1(new_n162));
  aboi22aa1n03x5               g067(.a(new_n162), .b(new_n151), .c(new_n157), .d(new_n161), .out0(\s[13] ));
  inv000aa1d42x5               g068(.a(new_n158), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n161), .b(new_n156), .c(new_n125), .d(new_n150), .o1(new_n165));
  norp02aa1n12x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nand02aa1n04x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nanb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(new_n168));
  xobna2aa1n03x5               g073(.a(new_n168), .b(new_n165), .c(new_n164), .out0(\s[14] ));
  nona23aa1d18x5               g074(.a(new_n167), .b(new_n159), .c(new_n158), .d(new_n166), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  aoai13aa1n03x5               g076(.a(new_n171), .b(new_n156), .c(new_n125), .d(new_n150), .o1(new_n172));
  oaoi03aa1n02x5               g077(.a(\a[14] ), .b(\b[13] ), .c(new_n164), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  nor042aa1n09x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanp02aa1n04x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  norb02aa1d27x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n177), .b(new_n172), .c(new_n174), .out0(\s[15] ));
  aob012aa1n03x5               g083(.a(new_n177), .b(new_n172), .c(new_n174), .out0(new_n179));
  nor042aa1n03x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nanp02aa1n03x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n181), .b(new_n180), .out0(new_n182));
  norp02aa1n02x5               g087(.a(new_n182), .b(new_n175), .o1(new_n183));
  inv020aa1n03x5               g088(.a(new_n175), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n177), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(new_n172), .d(new_n174), .o1(new_n186));
  aoi022aa1n03x5               g091(.a(new_n186), .b(new_n182), .c(new_n179), .d(new_n183), .o1(\s[16] ));
  nona23aa1n09x5               g092(.a(new_n181), .b(new_n176), .c(new_n175), .d(new_n180), .out0(new_n188));
  nor042aa1n09x5               g093(.a(new_n188), .b(new_n170), .o1(new_n189));
  nona32aa1n09x5               g094(.a(new_n189), .b(new_n135), .c(new_n143), .d(new_n137), .out0(new_n190));
  nanb02aa1n02x5               g095(.a(new_n190), .b(new_n125), .out0(new_n191));
  xorc02aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  nanb03aa1n06x5               g097(.a(new_n180), .b(new_n181), .c(new_n176), .out0(new_n193));
  oai112aa1n03x5               g098(.a(new_n184), .b(new_n167), .c(new_n166), .d(new_n158), .o1(new_n194));
  norp02aa1n02x5               g099(.a(new_n194), .b(new_n193), .o1(new_n195));
  tech160nm_fiaoi012aa1n03p5x5 g100(.a(new_n180), .b(new_n175), .c(new_n181), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n192), .b(new_n196), .out0(new_n197));
  aoi112aa1n02x5               g102(.a(new_n197), .b(new_n195), .c(new_n156), .d(new_n189), .o1(new_n198));
  oai012aa1n03x5               g103(.a(new_n196), .b(new_n194), .c(new_n193), .o1(new_n199));
  aoi012aa1n06x5               g104(.a(new_n199), .b(new_n156), .c(new_n189), .o1(new_n200));
  aoai13aa1n12x5               g105(.a(new_n200), .b(new_n190), .c(new_n118), .d(new_n124), .o1(new_n201));
  aoi022aa1n02x5               g106(.a(new_n201), .b(new_n192), .c(new_n191), .d(new_n198), .o1(\s[17] ));
  nor042aa1n03x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  inv000aa1d42x5               g109(.a(\a[17] ), .o1(new_n205));
  oaib12aa1n02x5               g110(.a(new_n201), .b(new_n205), .c(\b[16] ), .out0(new_n206));
  xorc02aa1n02x5               g111(.a(\a[18] ), .b(\b[17] ), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n206), .c(new_n204), .out0(\s[18] ));
  inv000aa1d42x5               g113(.a(\a[18] ), .o1(new_n209));
  xroi22aa1d06x4               g114(.a(new_n205), .b(\b[16] ), .c(new_n209), .d(\b[17] ), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n201), .b(new_n210), .o1(new_n211));
  oao003aa1n03x5               g116(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .carry(new_n212));
  tech160nm_fixorc02aa1n05x5   g117(.a(\a[19] ), .b(\b[18] ), .out0(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n213), .b(new_n211), .c(new_n212), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n03x5               g120(.a(new_n213), .b(new_n211), .c(new_n212), .out0(new_n216));
  nor002aa1n04x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  and002aa1n12x5               g122(.a(\b[19] ), .b(\a[20] ), .o(new_n218));
  nor022aa1n04x5               g123(.a(new_n218), .b(new_n217), .o1(new_n219));
  nor002aa1n03x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  oab012aa1n02x4               g125(.a(new_n220), .b(new_n218), .c(new_n217), .out0(new_n221));
  aobi12aa1n02x5               g126(.a(new_n212), .b(new_n201), .c(new_n210), .out0(new_n222));
  oaoi03aa1n02x5               g127(.a(\a[19] ), .b(\b[18] ), .c(new_n222), .o1(new_n223));
  aoi022aa1n02x5               g128(.a(new_n223), .b(new_n219), .c(new_n216), .d(new_n221), .o1(\s[20] ));
  nand23aa1n04x5               g129(.a(new_n210), .b(new_n213), .c(new_n219), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  oai022aa1n02x5               g131(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n227));
  aoi112aa1n03x5               g132(.a(new_n218), .b(new_n217), .c(\a[19] ), .d(\b[18] ), .o1(new_n228));
  aoi012aa1n02x5               g133(.a(new_n220), .b(\a[18] ), .c(\b[17] ), .o1(new_n229));
  nand23aa1n03x5               g134(.a(new_n228), .b(new_n227), .c(new_n229), .o1(new_n230));
  aoib12aa1n06x5               g135(.a(new_n217), .b(new_n220), .c(new_n218), .out0(new_n231));
  nanp02aa1n02x5               g136(.a(new_n230), .b(new_n231), .o1(new_n232));
  xorc02aa1n12x5               g137(.a(\a[21] ), .b(\b[20] ), .out0(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n232), .c(new_n201), .d(new_n226), .o1(new_n234));
  nano22aa1n02x4               g139(.a(new_n233), .b(new_n230), .c(new_n231), .out0(new_n235));
  aobi12aa1n02x5               g140(.a(new_n235), .b(new_n201), .c(new_n226), .out0(new_n236));
  norb02aa1n02x5               g141(.a(new_n234), .b(new_n236), .out0(\s[21] ));
  xorc02aa1n12x5               g142(.a(\a[22] ), .b(\b[21] ), .out0(new_n238));
  norp02aa1n02x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  norp02aa1n02x5               g144(.a(new_n238), .b(new_n239), .o1(new_n240));
  inv000aa1d42x5               g145(.a(\a[21] ), .o1(new_n241));
  oaib12aa1n03x5               g146(.a(new_n234), .b(\b[20] ), .c(new_n241), .out0(new_n242));
  aoi022aa1n02x5               g147(.a(new_n242), .b(new_n238), .c(new_n234), .d(new_n240), .o1(\s[22] ));
  nand02aa1d06x5               g148(.a(new_n238), .b(new_n233), .o1(new_n244));
  nano32aa1n02x4               g149(.a(new_n244), .b(new_n210), .c(new_n213), .d(new_n219), .out0(new_n245));
  oai022aa1n02x5               g150(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n246));
  aob012aa1n02x5               g151(.a(new_n246), .b(\b[21] ), .c(\a[22] ), .out0(new_n247));
  aoai13aa1n12x5               g152(.a(new_n247), .b(new_n244), .c(new_n230), .d(new_n231), .o1(new_n248));
  tech160nm_fixorc02aa1n03p5x5 g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n201), .d(new_n245), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n244), .o1(new_n251));
  nanb02aa1n02x5               g156(.a(new_n249), .b(new_n247), .out0(new_n252));
  aoi012aa1n02x5               g157(.a(new_n252), .b(new_n232), .c(new_n251), .o1(new_n253));
  aobi12aa1n02x5               g158(.a(new_n253), .b(new_n201), .c(new_n245), .out0(new_n254));
  norb02aa1n02x5               g159(.a(new_n250), .b(new_n254), .out0(\s[23] ));
  xorc02aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .out0(new_n256));
  norp02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  norp02aa1n02x5               g162(.a(new_n256), .b(new_n257), .o1(new_n258));
  tech160nm_fioai012aa1n03p5x5 g163(.a(new_n250), .b(\b[22] ), .c(\a[23] ), .o1(new_n259));
  aoi022aa1n02x5               g164(.a(new_n259), .b(new_n256), .c(new_n250), .d(new_n258), .o1(\s[24] ));
  and002aa1n02x5               g165(.a(new_n256), .b(new_n249), .o(new_n261));
  nano22aa1n02x5               g166(.a(new_n225), .b(new_n251), .c(new_n261), .out0(new_n262));
  nand02aa1d06x5               g167(.a(new_n201), .b(new_n262), .o1(new_n263));
  inv000aa1n03x5               g168(.a(new_n257), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[24] ), .b(\b[23] ), .c(new_n264), .carry(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoi012aa1n02x5               g171(.a(new_n266), .b(new_n248), .c(new_n261), .o1(new_n267));
  xorc02aa1n12x5               g172(.a(\a[25] ), .b(\b[24] ), .out0(new_n268));
  aob012aa1n03x5               g173(.a(new_n268), .b(new_n263), .c(new_n267), .out0(new_n269));
  aoi112aa1n02x5               g174(.a(new_n268), .b(new_n266), .c(new_n248), .d(new_n261), .o1(new_n270));
  aobi12aa1n03x7               g175(.a(new_n269), .b(new_n270), .c(new_n263), .out0(\s[25] ));
  xorc02aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .out0(new_n272));
  nor042aa1n03x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  norp02aa1n02x5               g178(.a(new_n272), .b(new_n273), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n273), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n268), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n275), .b(new_n276), .c(new_n263), .d(new_n267), .o1(new_n277));
  aoi022aa1n02x7               g182(.a(new_n277), .b(new_n272), .c(new_n269), .d(new_n274), .o1(\s[26] ));
  and002aa1n02x5               g183(.a(new_n272), .b(new_n268), .o(new_n279));
  aoai13aa1n09x5               g184(.a(new_n279), .b(new_n266), .c(new_n248), .d(new_n261), .o1(new_n280));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  and002aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o(new_n282));
  norp02aa1n02x5               g187(.a(new_n282), .b(new_n281), .o1(new_n283));
  nano32aa1n03x7               g188(.a(new_n225), .b(new_n279), .c(new_n251), .d(new_n261), .out0(new_n284));
  oao003aa1n02x5               g189(.a(\a[26] ), .b(\b[25] ), .c(new_n275), .carry(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  aoi112aa1n02x5               g191(.a(new_n286), .b(new_n283), .c(new_n201), .d(new_n284), .o1(new_n287));
  nand02aa1d06x5               g192(.a(new_n201), .b(new_n284), .o1(new_n288));
  nand23aa1n06x5               g193(.a(new_n288), .b(new_n280), .c(new_n285), .o1(new_n289));
  aoi022aa1n02x5               g194(.a(new_n289), .b(new_n283), .c(new_n287), .d(new_n280), .o1(\s[27] ));
  inv000aa1d42x5               g195(.a(\b[26] ), .o1(new_n291));
  oaib12aa1n03x5               g196(.a(new_n289), .b(new_n291), .c(\a[27] ), .out0(new_n292));
  inv000aa1n03x5               g197(.a(new_n281), .o1(new_n293));
  tech160nm_fiaoi012aa1n05x5   g198(.a(new_n286), .b(new_n201), .c(new_n284), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n293), .b(new_n282), .c(new_n294), .d(new_n280), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n296), .b(new_n281), .o1(new_n297));
  aoi022aa1n03x5               g202(.a(new_n295), .b(new_n296), .c(new_n292), .d(new_n297), .o1(\s[28] ));
  inv000aa1d42x5               g203(.a(\a[28] ), .o1(new_n299));
  xroi22aa1d04x5               g204(.a(\a[27] ), .b(new_n291), .c(new_n299), .d(\b[27] ), .out0(new_n300));
  nand02aa1n03x5               g205(.a(new_n289), .b(new_n300), .o1(new_n301));
  inv000aa1n03x5               g206(.a(new_n300), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .carry(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n302), .c(new_n294), .d(new_n280), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .out0(new_n305));
  norb02aa1n02x5               g210(.a(new_n303), .b(new_n305), .out0(new_n306));
  aoi022aa1n03x5               g211(.a(new_n304), .b(new_n305), .c(new_n301), .d(new_n306), .o1(\s[29] ));
  xorb03aa1n02x5               g212(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g213(.a(new_n296), .b(new_n305), .c(new_n283), .o(new_n309));
  nand22aa1n03x5               g214(.a(new_n289), .b(new_n309), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n309), .o1(new_n311));
  inv000aa1d42x5               g216(.a(\b[28] ), .o1(new_n312));
  inv000aa1d42x5               g217(.a(\a[29] ), .o1(new_n313));
  oaib12aa1n02x5               g218(.a(new_n303), .b(\b[28] ), .c(new_n313), .out0(new_n314));
  oaib12aa1n02x5               g219(.a(new_n314), .b(new_n312), .c(\a[29] ), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n311), .c(new_n294), .d(new_n280), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[30] ), .b(\b[29] ), .out0(new_n317));
  oaoi13aa1n02x5               g222(.a(new_n317), .b(new_n314), .c(new_n313), .d(new_n312), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n316), .b(new_n317), .c(new_n310), .d(new_n318), .o1(\s[30] ));
  nano22aa1n02x4               g224(.a(new_n302), .b(new_n305), .c(new_n317), .out0(new_n320));
  nanp02aa1n03x5               g225(.a(new_n289), .b(new_n320), .o1(new_n321));
  aoi022aa1n02x5               g226(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n322));
  norb02aa1n02x5               g227(.a(\b[30] ), .b(\a[31] ), .out0(new_n323));
  obai22aa1n02x7               g228(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n324));
  aoi112aa1n02x5               g229(.a(new_n324), .b(new_n323), .c(new_n314), .d(new_n322), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[31] ), .b(\b[30] ), .out0(new_n326));
  inv000aa1n02x5               g231(.a(new_n320), .o1(new_n327));
  norp02aa1n02x5               g232(.a(\b[29] ), .b(\a[30] ), .o1(new_n328));
  aoi012aa1n02x5               g233(.a(new_n328), .b(new_n314), .c(new_n322), .o1(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n327), .c(new_n294), .d(new_n280), .o1(new_n330));
  aoi022aa1n03x5               g235(.a(new_n330), .b(new_n326), .c(new_n321), .d(new_n325), .o1(\s[31] ));
  xobna2aa1n03x5               g236(.a(new_n103), .b(new_n100), .c(new_n98), .out0(\s[3] ));
  nanp02aa1n02x5               g237(.a(new_n100), .b(new_n98), .o1(new_n333));
  nanp03aa1n02x5               g238(.a(new_n333), .b(new_n101), .c(new_n102), .o1(new_n334));
  xorc02aa1n02x5               g239(.a(\a[4] ), .b(\b[3] ), .out0(new_n335));
  xnbna2aa1n03x5               g240(.a(new_n335), .b(new_n334), .c(new_n101), .out0(\s[4] ));
  nanp02aa1n02x5               g241(.a(\b[3] ), .b(\a[4] ), .o1(new_n337));
  norb02aa1n02x5               g242(.a(new_n106), .b(new_n107), .out0(new_n338));
  xobna2aa1n03x5               g243(.a(new_n338), .b(new_n105), .c(new_n337), .out0(\s[5] ));
  aob012aa1n02x5               g244(.a(new_n338), .b(new_n105), .c(new_n337), .out0(new_n340));
  norb02aa1n02x5               g245(.a(new_n119), .b(new_n115), .out0(new_n341));
  xobna2aa1n03x5               g246(.a(new_n341), .b(new_n340), .c(new_n106), .out0(\s[6] ));
  norb02aa1n02x5               g247(.a(new_n114), .b(new_n113), .out0(new_n343));
  aob012aa1n02x5               g248(.a(new_n341), .b(new_n340), .c(new_n106), .out0(new_n344));
  xobna2aa1n03x5               g249(.a(new_n343), .b(new_n344), .c(new_n119), .out0(\s[7] ));
  aoai13aa1n02x5               g250(.a(new_n122), .b(new_n113), .c(new_n344), .d(new_n120), .o1(new_n346));
  aoi112aa1n02x5               g251(.a(new_n122), .b(new_n113), .c(new_n344), .d(new_n120), .o1(new_n347));
  norb02aa1n02x5               g252(.a(new_n346), .b(new_n347), .out0(\s[8] ));
  aoi113aa1n02x5               g253(.a(new_n123), .b(new_n131), .c(new_n120), .d(new_n122), .e(new_n121), .o1(new_n349));
  aoi022aa1n02x5               g254(.a(new_n125), .b(new_n131), .c(new_n118), .d(new_n349), .o1(\s[9] ));
endmodule


