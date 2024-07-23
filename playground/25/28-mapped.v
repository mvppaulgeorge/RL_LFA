// Benchmark "adder" written by ABC on Thu Jul 18 01:01:08 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n328, new_n329, new_n332, new_n333,
    new_n336, new_n337, new_n338;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n04x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand02aa1d06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d10x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aoi012aa1d18x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n24x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand42aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n02x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand02aa1d06x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n02x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nanb03aa1n02x5               g012(.a(new_n101), .b(new_n107), .c(new_n104), .out0(new_n108));
  inv040aa1d32x5               g013(.a(\a[3] ), .o1(new_n109));
  inv040aa1n20x5               g014(.a(\b[2] ), .o1(new_n110));
  nand02aa1d24x5               g015(.a(new_n110), .b(new_n109), .o1(new_n111));
  oaoi03aa1n09x5               g016(.a(\a[4] ), .b(\b[3] ), .c(new_n111), .o1(new_n112));
  inv030aa1n02x5               g017(.a(new_n112), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand02aa1d08x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor042aa1n04x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nanp02aa1n04x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nano23aa1n02x4               g022(.a(new_n114), .b(new_n116), .c(new_n117), .d(new_n115), .out0(new_n118));
  nor042aa1n04x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nand02aa1n08x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nor042aa1n06x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nanp02aa1n04x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  nano23aa1n03x7               g027(.a(new_n119), .b(new_n121), .c(new_n122), .d(new_n120), .out0(new_n123));
  nand02aa1n02x5               g028(.a(new_n123), .b(new_n118), .o1(new_n124));
  inv030aa1n02x5               g029(.a(new_n114), .o1(new_n125));
  aob012aa1n06x5               g030(.a(new_n125), .b(new_n116), .c(new_n115), .out0(new_n126));
  oai022aa1n02x5               g031(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n127));
  aoi022aa1n12x5               g032(.a(new_n123), .b(new_n126), .c(new_n120), .d(new_n127), .o1(new_n128));
  aoai13aa1n12x5               g033(.a(new_n128), .b(new_n124), .c(new_n108), .d(new_n113), .o1(new_n129));
  nand22aa1n04x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  aoi012aa1n02x5               g035(.a(new_n97), .b(new_n129), .c(new_n130), .o1(new_n131));
  xnrb03aa1n02x5               g036(.a(new_n131), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1d18x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nand22aa1n12x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nano23aa1n06x5               g039(.a(new_n97), .b(new_n133), .c(new_n134), .d(new_n130), .out0(new_n135));
  aoi012aa1d24x5               g040(.a(new_n133), .b(new_n97), .c(new_n134), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  nor002aa1d32x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nanp02aa1n04x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n137), .c(new_n129), .d(new_n135), .o1(new_n141));
  aoi112aa1n02x5               g046(.a(new_n140), .b(new_n137), .c(new_n129), .d(new_n135), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(\s[11] ));
  oai012aa1n02x5               g048(.a(new_n141), .b(\b[10] ), .c(\a[11] ), .o1(new_n144));
  nor002aa1d32x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n06x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  aoib12aa1n02x5               g052(.a(new_n138), .b(new_n146), .c(new_n145), .out0(new_n148));
  aoi022aa1n02x5               g053(.a(new_n144), .b(new_n147), .c(new_n141), .d(new_n148), .o1(\s[12] ));
  nona23aa1n06x5               g054(.a(new_n146), .b(new_n139), .c(new_n138), .d(new_n145), .out0(new_n150));
  norb02aa1n02x5               g055(.a(new_n135), .b(new_n150), .out0(new_n151));
  oaih12aa1n02x5               g056(.a(new_n146), .b(new_n145), .c(new_n138), .o1(new_n152));
  oai012aa1n06x5               g057(.a(new_n152), .b(new_n150), .c(new_n136), .o1(new_n153));
  ao0012aa1n03x7               g058(.a(new_n153), .b(new_n129), .c(new_n151), .o(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand22aa1n09x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nor002aa1d32x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand02aa1d28x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  aoai13aa1n03x5               g065(.a(new_n160), .b(new_n156), .c(new_n154), .d(new_n157), .o1(new_n161));
  aoi112aa1n02x5               g066(.a(new_n156), .b(new_n160), .c(new_n154), .d(new_n157), .o1(new_n162));
  norb02aa1n03x4               g067(.a(new_n161), .b(new_n162), .out0(\s[14] ));
  nona23aa1d24x5               g068(.a(new_n159), .b(new_n157), .c(new_n156), .d(new_n158), .out0(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n06x5               g070(.a(new_n165), .b(new_n153), .c(new_n129), .d(new_n151), .o1(new_n166));
  aoi012aa1d18x5               g071(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n167));
  nor002aa1d32x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand22aa1n04x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  xobna2aa1n03x5               g075(.a(new_n170), .b(new_n166), .c(new_n167), .out0(\s[15] ));
  tech160nm_fiao0012aa1n02p5x5 g076(.a(new_n170), .b(new_n166), .c(new_n167), .o(new_n172));
  inv000aa1d42x5               g077(.a(new_n168), .o1(new_n173));
  aoai13aa1n02x7               g078(.a(new_n173), .b(new_n170), .c(new_n166), .d(new_n167), .o1(new_n174));
  nor002aa1d32x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand02aa1d06x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  aoib12aa1n02x5               g082(.a(new_n168), .b(new_n176), .c(new_n175), .out0(new_n178));
  aoi022aa1n03x5               g083(.a(new_n172), .b(new_n178), .c(new_n174), .d(new_n177), .o1(\s[16] ));
  nanb02aa1n06x5               g084(.a(new_n102), .b(new_n103), .out0(new_n180));
  nand02aa1n04x5               g085(.a(new_n111), .b(new_n106), .o1(new_n181));
  norp03aa1n02x5               g086(.a(new_n101), .b(new_n180), .c(new_n181), .o1(new_n182));
  nona23aa1n09x5               g087(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n183));
  nona23aa1n02x4               g088(.a(new_n122), .b(new_n120), .c(new_n119), .d(new_n121), .out0(new_n184));
  nor042aa1n02x5               g089(.a(new_n184), .b(new_n183), .o1(new_n185));
  tech160nm_fioai012aa1n05x5   g090(.a(new_n185), .b(new_n182), .c(new_n112), .o1(new_n186));
  nona23aa1d18x5               g091(.a(new_n176), .b(new_n169), .c(new_n168), .d(new_n175), .out0(new_n187));
  inv040aa1n02x5               g092(.a(new_n187), .o1(new_n188));
  nona23aa1n09x5               g093(.a(new_n135), .b(new_n188), .c(new_n164), .d(new_n150), .out0(new_n189));
  nor042aa1n02x5               g094(.a(new_n187), .b(new_n164), .o1(new_n190));
  tech160nm_fioai012aa1n03p5x5 g095(.a(new_n176), .b(new_n175), .c(new_n168), .o1(new_n191));
  oai012aa1d24x5               g096(.a(new_n191), .b(new_n187), .c(new_n167), .o1(new_n192));
  aoi012aa1n09x5               g097(.a(new_n192), .b(new_n153), .c(new_n190), .o1(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n189), .c(new_n186), .d(new_n128), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1n04x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  tech160nm_fixorc02aa1n03p5x5 g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  nor042aa1n04x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nanp02aa1n12x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  norb02aa1n06x4               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  aoai13aa1n02x7               g105(.a(new_n200), .b(new_n196), .c(new_n194), .d(new_n197), .o1(new_n201));
  aoi112aa1n02x5               g106(.a(new_n196), .b(new_n200), .c(new_n194), .d(new_n197), .o1(new_n202));
  norb02aa1n03x4               g107(.a(new_n201), .b(new_n202), .out0(\s[18] ));
  and002aa1n06x5               g108(.a(new_n197), .b(new_n200), .o(new_n204));
  aoi012aa1d18x5               g109(.a(new_n198), .b(new_n196), .c(new_n199), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  xnrc02aa1n12x5               g111(.a(\b[18] ), .b(\a[19] ), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n206), .c(new_n194), .d(new_n204), .o1(new_n209));
  aoi112aa1n02x5               g114(.a(new_n208), .b(new_n206), .c(new_n194), .d(new_n204), .o1(new_n210));
  norb02aa1n03x4               g115(.a(new_n209), .b(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  inv000aa1n02x5               g118(.a(new_n213), .o1(new_n214));
  nanp02aa1n03x5               g119(.a(new_n209), .b(new_n214), .o1(new_n215));
  xnrc02aa1n12x5               g120(.a(\b[19] ), .b(\a[20] ), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n216), .b(new_n213), .out0(new_n218));
  aoi022aa1n02x7               g123(.a(new_n215), .b(new_n217), .c(new_n209), .d(new_n218), .o1(\s[20] ));
  nano23aa1n02x4               g124(.a(new_n216), .b(new_n207), .c(new_n197), .d(new_n200), .out0(new_n220));
  oao003aa1n03x5               g125(.a(\a[20] ), .b(\b[19] ), .c(new_n214), .carry(new_n221));
  oai013aa1d12x5               g126(.a(new_n221), .b(new_n207), .c(new_n216), .d(new_n205), .o1(new_n222));
  nor042aa1n06x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  nand22aa1n09x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  nanb02aa1n12x5               g129(.a(new_n223), .b(new_n224), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n222), .c(new_n194), .d(new_n220), .o1(new_n227));
  aoi112aa1n02x5               g132(.a(new_n226), .b(new_n222), .c(new_n194), .d(new_n220), .o1(new_n228));
  norb02aa1n03x4               g133(.a(new_n227), .b(new_n228), .out0(\s[21] ));
  oai012aa1n06x5               g134(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .o1(new_n230));
  nor022aa1n12x5               g135(.a(\b[21] ), .b(\a[22] ), .o1(new_n231));
  nand02aa1n08x5               g136(.a(\b[21] ), .b(\a[22] ), .o1(new_n232));
  nanb02aa1n09x5               g137(.a(new_n231), .b(new_n232), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoib12aa1n02x5               g139(.a(new_n223), .b(new_n232), .c(new_n231), .out0(new_n235));
  aoi022aa1n02x7               g140(.a(new_n230), .b(new_n234), .c(new_n227), .d(new_n235), .o1(\s[22] ));
  nor042aa1n02x5               g141(.a(new_n216), .b(new_n207), .o1(new_n237));
  nona23aa1n09x5               g142(.a(new_n232), .b(new_n224), .c(new_n223), .d(new_n231), .out0(new_n238));
  nano32aa1n02x4               g143(.a(new_n238), .b(new_n237), .c(new_n200), .d(new_n197), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n238), .o1(new_n240));
  tech160nm_fiao0012aa1n05x5   g145(.a(new_n231), .b(new_n223), .c(new_n232), .o(new_n241));
  tech160nm_fiao0012aa1n02p5x5 g146(.a(new_n241), .b(new_n222), .c(new_n240), .o(new_n242));
  tech160nm_fixnrc02aa1n04x5   g147(.a(\b[22] ), .b(\a[23] ), .out0(new_n243));
  inv000aa1n03x5               g148(.a(new_n243), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n242), .c(new_n194), .d(new_n239), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(new_n244), .b(new_n242), .c(new_n194), .d(new_n239), .o1(new_n246));
  norb02aa1n03x4               g151(.a(new_n245), .b(new_n246), .out0(\s[23] ));
  oai012aa1n06x5               g152(.a(new_n245), .b(\b[22] ), .c(\a[23] ), .o1(new_n248));
  tech160nm_fixorc02aa1n05x5   g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  norp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  aoi022aa1n02x7               g156(.a(new_n248), .b(new_n249), .c(new_n245), .d(new_n251), .o1(\s[24] ));
  nona23aa1n09x5               g157(.a(new_n244), .b(new_n249), .c(new_n233), .d(new_n225), .out0(new_n253));
  nano32aa1n02x4               g158(.a(new_n253), .b(new_n237), .c(new_n200), .d(new_n197), .out0(new_n254));
  and002aa1n02x5               g159(.a(new_n194), .b(new_n254), .o(new_n255));
  nona22aa1n02x4               g160(.a(new_n217), .b(new_n207), .c(new_n205), .out0(new_n256));
  aob012aa1n02x5               g161(.a(new_n250), .b(\b[23] ), .c(\a[24] ), .out0(new_n257));
  oai012aa1n02x5               g162(.a(new_n257), .b(\b[23] ), .c(\a[24] ), .o1(new_n258));
  aoi013aa1n03x5               g163(.a(new_n258), .b(new_n244), .c(new_n241), .d(new_n249), .o1(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n253), .c(new_n256), .d(new_n221), .o1(new_n260));
  xorc02aa1n12x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n260), .c(new_n194), .d(new_n254), .o1(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[23] ), .b(\a[24] ), .out0(new_n263));
  nor043aa1n03x5               g168(.a(new_n238), .b(new_n243), .c(new_n263), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n222), .b(new_n264), .o1(new_n265));
  nanb03aa1n02x5               g170(.a(new_n261), .b(new_n265), .c(new_n259), .out0(new_n266));
  oa0012aa1n03x5               g171(.a(new_n262), .b(new_n255), .c(new_n266), .o(\s[25] ));
  inv000aa1d42x5               g172(.a(\a[25] ), .o1(new_n268));
  oaib12aa1n06x5               g173(.a(new_n262), .b(\b[24] ), .c(new_n268), .out0(new_n269));
  tech160nm_fixorc02aa1n03p5x5 g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  aoib12aa1n02x5               g175(.a(new_n270), .b(new_n268), .c(\b[24] ), .out0(new_n271));
  aoi022aa1n02x7               g176(.a(new_n269), .b(new_n270), .c(new_n262), .d(new_n271), .o1(\s[26] ));
  nano23aa1n02x4               g177(.a(new_n138), .b(new_n145), .c(new_n146), .d(new_n139), .out0(new_n273));
  nano23aa1n02x4               g178(.a(new_n187), .b(new_n164), .c(new_n273), .d(new_n135), .out0(new_n274));
  nanp02aa1n02x5               g179(.a(new_n153), .b(new_n190), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n192), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(new_n275), .b(new_n276), .o1(new_n277));
  and002aa1n06x5               g182(.a(new_n270), .b(new_n261), .o(new_n278));
  nano32aa1n03x7               g183(.a(new_n253), .b(new_n278), .c(new_n204), .d(new_n237), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n277), .c(new_n129), .d(new_n274), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(\b[25] ), .b(\a[26] ), .o1(new_n281));
  oai022aa1n02x5               g186(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n282));
  aoi022aa1n06x5               g187(.a(new_n260), .b(new_n278), .c(new_n281), .d(new_n282), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[27] ), .b(\b[26] ), .out0(new_n284));
  xnbna2aa1n03x5               g189(.a(new_n284), .b(new_n280), .c(new_n283), .out0(\s[27] ));
  inv000aa1d42x5               g190(.a(new_n278), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(new_n282), .b(new_n281), .o1(new_n287));
  aoai13aa1n04x5               g192(.a(new_n287), .b(new_n286), .c(new_n265), .d(new_n259), .o1(new_n288));
  aoai13aa1n06x5               g193(.a(new_n284), .b(new_n288), .c(new_n194), .d(new_n279), .o1(new_n289));
  inv000aa1d42x5               g194(.a(\a[27] ), .o1(new_n290));
  oaib12aa1n06x5               g195(.a(new_n289), .b(\b[26] ), .c(new_n290), .out0(new_n291));
  xorc02aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .out0(new_n292));
  norp02aa1n02x5               g197(.a(\b[26] ), .b(\a[27] ), .o1(new_n293));
  norp02aa1n02x5               g198(.a(new_n292), .b(new_n293), .o1(new_n294));
  aoi022aa1n02x7               g199(.a(new_n291), .b(new_n292), .c(new_n289), .d(new_n294), .o1(\s[28] ));
  inv000aa1d42x5               g200(.a(\a[28] ), .o1(new_n296));
  xroi22aa1d04x5               g201(.a(new_n290), .b(\b[26] ), .c(new_n296), .d(\b[27] ), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n288), .c(new_n194), .d(new_n279), .o1(new_n298));
  inv000aa1d42x5               g203(.a(\b[27] ), .o1(new_n299));
  oao003aa1n09x5               g204(.a(new_n296), .b(new_n299), .c(new_n293), .carry(new_n300));
  inv000aa1d42x5               g205(.a(new_n300), .o1(new_n301));
  nanp02aa1n03x5               g206(.a(new_n298), .b(new_n301), .o1(new_n302));
  xorc02aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .out0(new_n303));
  norp02aa1n02x5               g208(.a(new_n300), .b(new_n303), .o1(new_n304));
  aoi022aa1n02x7               g209(.a(new_n302), .b(new_n303), .c(new_n298), .d(new_n304), .o1(\s[29] ));
  xorb03aa1n02x5               g210(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g211(.a(new_n284), .b(new_n303), .c(new_n292), .o(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n288), .c(new_n194), .d(new_n279), .o1(new_n308));
  inv000aa1d42x5               g213(.a(\a[29] ), .o1(new_n309));
  inv000aa1d42x5               g214(.a(\b[28] ), .o1(new_n310));
  oaoi03aa1n02x5               g215(.a(new_n309), .b(new_n310), .c(new_n300), .o1(new_n311));
  nanp02aa1n03x5               g216(.a(new_n308), .b(new_n311), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .out0(new_n313));
  oabi12aa1n02x5               g218(.a(new_n313), .b(\a[29] ), .c(\b[28] ), .out0(new_n314));
  oaoi13aa1n02x5               g219(.a(new_n314), .b(new_n300), .c(new_n309), .d(new_n310), .o1(new_n315));
  aoi022aa1n02x7               g220(.a(new_n312), .b(new_n313), .c(new_n308), .d(new_n315), .o1(\s[30] ));
  and003aa1n02x5               g221(.a(new_n297), .b(new_n313), .c(new_n303), .o(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n288), .c(new_n194), .d(new_n279), .o1(new_n318));
  oao003aa1n02x5               g223(.a(\a[30] ), .b(\b[29] ), .c(new_n311), .carry(new_n319));
  nanb02aa1n02x5               g224(.a(\b[30] ), .b(\a[31] ), .out0(new_n320));
  nanb02aa1n02x5               g225(.a(\a[31] ), .b(\b[30] ), .out0(new_n321));
  aoi022aa1n03x5               g226(.a(new_n318), .b(new_n319), .c(new_n321), .d(new_n320), .o1(new_n322));
  aobi12aa1n02x7               g227(.a(new_n317), .b(new_n280), .c(new_n283), .out0(new_n323));
  nano32aa1n02x4               g228(.a(new_n323), .b(new_n321), .c(new_n319), .d(new_n320), .out0(new_n324));
  nor002aa1n02x5               g229(.a(new_n322), .b(new_n324), .o1(\s[31] ));
  xnbna2aa1n03x5               g230(.a(new_n101), .b(new_n106), .c(new_n111), .out0(\s[3] ));
  oai013aa1n03x5               g231(.a(new_n113), .b(new_n101), .c(new_n180), .d(new_n181), .o1(new_n327));
  oaib12aa1n02x5               g232(.a(new_n111), .b(new_n102), .c(new_n103), .out0(new_n328));
  oab012aa1n02x4               g233(.a(new_n328), .b(new_n101), .c(new_n181), .out0(new_n329));
  oaoi13aa1n02x5               g234(.a(new_n329), .b(new_n327), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g235(.a(new_n327), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi122aa1n02x5               g236(.a(new_n116), .b(new_n125), .c(new_n115), .d(new_n327), .e(new_n117), .o1(new_n332));
  tech160nm_fiao0012aa1n03p5x5 g237(.a(new_n126), .b(new_n327), .c(new_n118), .o(new_n333));
  aoi012aa1n02x5               g238(.a(new_n332), .b(new_n333), .c(new_n125), .o1(\s[6] ));
  xorb03aa1n02x5               g239(.a(new_n333), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g240(.a(new_n119), .b(new_n120), .out0(new_n336));
  orn002aa1n02x5               g241(.a(\a[7] ), .b(\b[6] ), .o(new_n337));
  nanb03aa1n02x5               g242(.a(new_n121), .b(new_n333), .c(new_n122), .out0(new_n338));
  xobna2aa1n03x5               g243(.a(new_n336), .b(new_n338), .c(new_n337), .out0(\s[8] ));
  xorb03aa1n02x5               g244(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


