// Benchmark "adder" written by ABC on Thu Jul 18 08:46:34 2024

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
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n335, new_n338, new_n339, new_n341,
    new_n343;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n06x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1d18x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor042aa1n09x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand02aa1d28x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  nand02aa1d28x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  aoi012aa1d24x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  inv020aa1n03x5               g009(.a(new_n104), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand22aa1n12x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanp02aa1n09x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nano23aa1n03x5               g014(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n110));
  inv040aa1n03x5               g015(.a(new_n106), .o1(new_n111));
  aob012aa1d15x5               g016(.a(new_n111), .b(new_n108), .c(new_n107), .out0(new_n112));
  aoi012aa1n02x7               g017(.a(new_n112), .b(new_n110), .c(new_n105), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand22aa1n12x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand22aa1n09x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nano23aa1n06x5               g022(.a(new_n114), .b(new_n116), .c(new_n117), .d(new_n115), .out0(new_n118));
  tech160nm_fixnrc02aa1n02p5x5 g023(.a(\b[5] ), .b(\a[6] ), .out0(new_n119));
  xnrc02aa1n03x5               g024(.a(\b[4] ), .b(\a[5] ), .out0(new_n120));
  nona22aa1n02x4               g025(.a(new_n118), .b(new_n120), .c(new_n119), .out0(new_n121));
  inv000aa1n02x5               g026(.a(new_n114), .o1(new_n122));
  aoi112aa1n09x5               g027(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n123));
  inv040aa1n03x5               g028(.a(new_n123), .o1(new_n124));
  nona23aa1d18x5               g029(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n125));
  nanp02aa1n02x5               g030(.a(\b[5] ), .b(\a[6] ), .o1(new_n126));
  oai022aa1d18x5               g031(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n127));
  nano22aa1n06x5               g032(.a(new_n125), .b(new_n126), .c(new_n127), .out0(new_n128));
  nano22aa1d15x5               g033(.a(new_n128), .b(new_n122), .c(new_n124), .out0(new_n129));
  tech160nm_fioai012aa1n05x5   g034(.a(new_n129), .b(new_n113), .c(new_n121), .o1(new_n130));
  xorc02aa1n12x5               g035(.a(\a[9] ), .b(\b[8] ), .out0(new_n131));
  aoi012aa1n02x5               g036(.a(new_n100), .b(new_n130), .c(new_n131), .o1(new_n132));
  xnrc02aa1n02x5               g037(.a(new_n132), .b(new_n99), .out0(\s[10] ));
  tech160nm_fiaoi012aa1n04x5   g038(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n134));
  inv020aa1n03x5               g039(.a(new_n134), .o1(new_n135));
  aoi013aa1n06x4               g040(.a(new_n135), .b(new_n130), .c(new_n131), .d(new_n99), .o1(new_n136));
  nor002aa1d32x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  nand02aa1d28x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n136), .b(new_n139), .c(new_n138), .out0(\s[11] ));
  norb02aa1d21x5               g045(.a(new_n139), .b(new_n137), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n141), .o1(new_n142));
  nor042aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanp02aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n06x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  inv040aa1n02x5               g050(.a(new_n145), .o1(new_n146));
  oaoi13aa1n06x5               g051(.a(new_n146), .b(new_n138), .c(new_n136), .d(new_n142), .o1(new_n147));
  oai112aa1n02x7               g052(.a(new_n146), .b(new_n138), .c(new_n136), .d(new_n142), .o1(new_n148));
  norb02aa1n03x4               g053(.a(new_n148), .b(new_n147), .out0(\s[12] ));
  nona23aa1n09x5               g054(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n150));
  oabi12aa1n18x5               g055(.a(new_n112), .b(new_n150), .c(new_n104), .out0(new_n151));
  nor043aa1n03x5               g056(.a(new_n125), .b(new_n119), .c(new_n120), .o1(new_n152));
  nand02aa1d04x5               g057(.a(new_n151), .b(new_n152), .o1(new_n153));
  nano32aa1n06x5               g058(.a(new_n146), .b(new_n131), .c(new_n141), .d(new_n99), .out0(new_n154));
  inv000aa1n02x5               g059(.a(new_n154), .o1(new_n155));
  nano23aa1n06x5               g060(.a(new_n137), .b(new_n143), .c(new_n144), .d(new_n139), .out0(new_n156));
  aoi112aa1n09x5               g061(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n157));
  aoi112aa1n03x5               g062(.a(new_n157), .b(new_n143), .c(new_n156), .d(new_n135), .o1(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n155), .c(new_n153), .d(new_n129), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1d18x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand02aa1d24x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  aoi012aa1n03x5               g067(.a(new_n161), .b(new_n159), .c(new_n162), .o1(new_n163));
  xnrb03aa1n03x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n12x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nand02aa1d24x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nano23aa1n09x5               g071(.a(new_n161), .b(new_n165), .c(new_n166), .d(new_n162), .out0(new_n167));
  tech160nm_fiaoi012aa1n05x5   g072(.a(new_n165), .b(new_n161), .c(new_n166), .o1(new_n168));
  inv020aa1n04x5               g073(.a(new_n168), .o1(new_n169));
  nor002aa1d32x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nand22aa1n12x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  aoai13aa1n06x5               g077(.a(new_n172), .b(new_n169), .c(new_n159), .d(new_n167), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(new_n172), .b(new_n169), .c(new_n159), .d(new_n167), .o1(new_n174));
  norb02aa1n03x4               g079(.a(new_n173), .b(new_n174), .out0(\s[15] ));
  inv000aa1d42x5               g080(.a(new_n170), .o1(new_n176));
  norp02aa1n24x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand22aa1n09x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  aoi012aa1n02x7               g084(.a(new_n179), .b(new_n173), .c(new_n176), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n179), .o1(new_n181));
  nona22aa1n02x5               g086(.a(new_n173), .b(new_n181), .c(new_n170), .out0(new_n182));
  norb02aa1n03x4               g087(.a(new_n182), .b(new_n180), .out0(\s[16] ));
  nona23aa1n03x5               g088(.a(new_n178), .b(new_n171), .c(new_n170), .d(new_n177), .out0(new_n184));
  norb02aa1n03x5               g089(.a(new_n167), .b(new_n184), .out0(new_n185));
  nand02aa1n02x5               g090(.a(new_n154), .b(new_n185), .o1(new_n186));
  aoi112aa1n09x5               g091(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n187));
  oai112aa1n06x5               g092(.a(new_n141), .b(new_n145), .c(new_n187), .d(new_n97), .o1(new_n188));
  nona22aa1n03x5               g093(.a(new_n188), .b(new_n157), .c(new_n143), .out0(new_n189));
  aoi112aa1n02x5               g094(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n190));
  nano23aa1n06x5               g095(.a(new_n170), .b(new_n177), .c(new_n178), .d(new_n171), .out0(new_n191));
  nanp02aa1n02x5               g096(.a(new_n191), .b(new_n169), .o1(new_n192));
  nona22aa1n03x5               g097(.a(new_n192), .b(new_n190), .c(new_n177), .out0(new_n193));
  aoi012aa1n12x5               g098(.a(new_n193), .b(new_n189), .c(new_n185), .o1(new_n194));
  aoai13aa1n12x5               g099(.a(new_n194), .b(new_n186), .c(new_n153), .d(new_n129), .o1(new_n195));
  xorb03aa1n03x5               g100(.a(new_n195), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g101(.a(\a[18] ), .o1(new_n197));
  nor002aa1n02x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  xorc02aa1n02x5               g103(.a(\a[17] ), .b(\b[16] ), .out0(new_n199));
  tech160nm_fiaoi012aa1n05x5   g104(.a(new_n198), .b(new_n195), .c(new_n199), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[17] ), .c(new_n197), .out0(\s[18] ));
  nanp03aa1n02x5               g106(.a(new_n118), .b(new_n126), .c(new_n127), .o1(new_n202));
  nona22aa1n02x4               g107(.a(new_n202), .b(new_n123), .c(new_n114), .out0(new_n203));
  nanp02aa1n02x5               g108(.a(new_n191), .b(new_n167), .o1(new_n204));
  nano32aa1n03x7               g109(.a(new_n204), .b(new_n156), .c(new_n131), .d(new_n99), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n203), .c(new_n151), .d(new_n152), .o1(new_n206));
  inv000aa1d42x5               g111(.a(\a[17] ), .o1(new_n207));
  xroi22aa1d06x4               g112(.a(new_n207), .b(\b[16] ), .c(new_n197), .d(\b[17] ), .out0(new_n208));
  inv000aa1n02x5               g113(.a(new_n208), .o1(new_n209));
  inv000aa1d42x5               g114(.a(\b[17] ), .o1(new_n210));
  oao003aa1n02x5               g115(.a(new_n197), .b(new_n210), .c(new_n198), .carry(new_n211));
  inv000aa1n02x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n03x5               g117(.a(new_n212), .b(new_n209), .c(new_n206), .d(new_n194), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1d18x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  nand22aa1n09x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  norb02aa1n15x5               g123(.a(new_n218), .b(new_n216), .out0(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n211), .c(new_n195), .d(new_n208), .o1(new_n220));
  nor042aa1d18x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand02aa1d24x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n15x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  aobi12aa1n03x5               g128(.a(new_n223), .b(new_n220), .c(new_n217), .out0(new_n224));
  aoi112aa1n02x5               g129(.a(new_n216), .b(new_n223), .c(new_n213), .d(new_n218), .o1(new_n225));
  nor002aa1n02x5               g130(.a(new_n224), .b(new_n225), .o1(\s[20] ));
  nano23aa1n06x5               g131(.a(new_n216), .b(new_n221), .c(new_n222), .d(new_n218), .out0(new_n227));
  nand02aa1d04x5               g132(.a(new_n208), .b(new_n227), .o1(new_n228));
  aoi112aa1n09x5               g133(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n229));
  nor002aa1n03x5               g134(.a(\b[17] ), .b(\a[18] ), .o1(new_n230));
  aoi112aa1n09x5               g135(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n231));
  oai112aa1n06x5               g136(.a(new_n219), .b(new_n223), .c(new_n231), .d(new_n230), .o1(new_n232));
  nona22aa1d18x5               g137(.a(new_n232), .b(new_n229), .c(new_n221), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n228), .c(new_n206), .d(new_n194), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  inv020aa1n04x5               g142(.a(new_n237), .o1(new_n238));
  inv040aa1n03x5               g143(.a(new_n228), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[20] ), .b(\a[21] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n233), .c(new_n195), .d(new_n239), .o1(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[21] ), .b(\a[22] ), .out0(new_n243));
  aoi012aa1n03x5               g148(.a(new_n243), .b(new_n242), .c(new_n238), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n243), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(new_n237), .b(new_n245), .c(new_n235), .d(new_n241), .o1(new_n246));
  norp02aa1n03x5               g151(.a(new_n244), .b(new_n246), .o1(\s[22] ));
  nor042aa1n06x5               g152(.a(new_n243), .b(new_n240), .o1(new_n248));
  nand23aa1n06x5               g153(.a(new_n208), .b(new_n248), .c(new_n227), .o1(new_n249));
  oaoi03aa1n03x5               g154(.a(\a[22] ), .b(\b[21] ), .c(new_n238), .o1(new_n250));
  tech160nm_fiaoi012aa1n02p5x5 g155(.a(new_n250), .b(new_n233), .c(new_n248), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n249), .c(new_n206), .d(new_n194), .o1(new_n252));
  xorb03aa1n02x5               g157(.a(new_n252), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1d32x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  inv040aa1n06x5               g160(.a(new_n249), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n251), .o1(new_n257));
  nand02aa1d08x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  norb02aa1n06x4               g163(.a(new_n258), .b(new_n254), .out0(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n257), .c(new_n195), .d(new_n256), .o1(new_n260));
  nor002aa1n16x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  nand02aa1d04x5               g166(.a(\b[23] ), .b(\a[24] ), .o1(new_n262));
  norb02aa1n03x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  aobi12aa1n03x5               g168(.a(new_n263), .b(new_n260), .c(new_n255), .out0(new_n264));
  aoi112aa1n03x4               g169(.a(new_n254), .b(new_n263), .c(new_n252), .d(new_n259), .o1(new_n265));
  nor002aa1n02x5               g170(.a(new_n264), .b(new_n265), .o1(\s[24] ));
  nona23aa1n03x5               g171(.a(new_n262), .b(new_n258), .c(new_n254), .d(new_n261), .out0(new_n267));
  inv000aa1n02x5               g172(.a(new_n267), .o1(new_n268));
  nano22aa1n03x7               g173(.a(new_n228), .b(new_n248), .c(new_n268), .out0(new_n269));
  inv020aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  nona32aa1n09x5               g175(.a(new_n233), .b(new_n267), .c(new_n243), .d(new_n240), .out0(new_n271));
  aoi112aa1n02x5               g176(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n272));
  aoi113aa1n03x7               g177(.a(new_n272), .b(new_n261), .c(new_n250), .d(new_n263), .e(new_n259), .o1(new_n273));
  nand02aa1d06x5               g178(.a(new_n271), .b(new_n273), .o1(new_n274));
  inv040aa1n03x5               g179(.a(new_n274), .o1(new_n275));
  aoai13aa1n04x5               g180(.a(new_n275), .b(new_n270), .c(new_n206), .d(new_n194), .o1(new_n276));
  xorb03aa1n02x5               g181(.a(new_n276), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g182(.a(\b[24] ), .b(\a[25] ), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n278), .o1(new_n279));
  tech160nm_fixorc02aa1n05x5   g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n280), .b(new_n274), .c(new_n195), .d(new_n269), .o1(new_n281));
  xorc02aa1n03x5               g186(.a(\a[26] ), .b(\b[25] ), .out0(new_n282));
  aobi12aa1n03x5               g187(.a(new_n282), .b(new_n281), .c(new_n279), .out0(new_n283));
  aoi112aa1n03x4               g188(.a(new_n278), .b(new_n282), .c(new_n276), .d(new_n280), .o1(new_n284));
  nor002aa1n02x5               g189(.a(new_n283), .b(new_n284), .o1(\s[26] ));
  aoi112aa1n02x5               g190(.a(new_n190), .b(new_n177), .c(new_n191), .d(new_n169), .o1(new_n286));
  oai012aa1n02x5               g191(.a(new_n286), .b(new_n158), .c(new_n204), .o1(new_n287));
  and002aa1n06x5               g192(.a(new_n282), .b(new_n280), .o(new_n288));
  nano22aa1n03x7               g193(.a(new_n249), .b(new_n288), .c(new_n268), .out0(new_n289));
  aoai13aa1n06x5               g194(.a(new_n289), .b(new_n287), .c(new_n130), .d(new_n205), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[26] ), .b(\b[25] ), .c(new_n279), .carry(new_n291));
  aobi12aa1n09x5               g196(.a(new_n291), .b(new_n274), .c(new_n288), .out0(new_n292));
  xorc02aa1n12x5               g197(.a(\a[27] ), .b(\b[26] ), .out0(new_n293));
  xnbna2aa1n03x5               g198(.a(new_n293), .b(new_n292), .c(new_n290), .out0(\s[27] ));
  nor042aa1n03x5               g199(.a(\b[26] ), .b(\a[27] ), .o1(new_n295));
  inv040aa1n03x5               g200(.a(new_n295), .o1(new_n296));
  inv000aa1n02x5               g201(.a(new_n288), .o1(new_n297));
  aoai13aa1n04x5               g202(.a(new_n291), .b(new_n297), .c(new_n271), .d(new_n273), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n293), .b(new_n298), .c(new_n195), .d(new_n289), .o1(new_n299));
  xnrc02aa1n12x5               g204(.a(\b[27] ), .b(\a[28] ), .out0(new_n300));
  aoi012aa1n03x5               g205(.a(new_n300), .b(new_n299), .c(new_n296), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n293), .o1(new_n302));
  tech160nm_fiaoi012aa1n03p5x5 g207(.a(new_n302), .b(new_n292), .c(new_n290), .o1(new_n303));
  nano22aa1n02x5               g208(.a(new_n303), .b(new_n296), .c(new_n300), .out0(new_n304));
  nor002aa1n02x5               g209(.a(new_n301), .b(new_n304), .o1(\s[28] ));
  norb02aa1n03x4               g210(.a(new_n293), .b(new_n300), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n298), .c(new_n195), .d(new_n289), .o1(new_n307));
  oao003aa1n03x5               g212(.a(\a[28] ), .b(\b[27] ), .c(new_n296), .carry(new_n308));
  xnrc02aa1n12x5               g213(.a(\b[28] ), .b(\a[29] ), .out0(new_n309));
  aoi012aa1n03x5               g214(.a(new_n309), .b(new_n307), .c(new_n308), .o1(new_n310));
  inv000aa1n02x5               g215(.a(new_n306), .o1(new_n311));
  aoi012aa1n02x7               g216(.a(new_n311), .b(new_n292), .c(new_n290), .o1(new_n312));
  nano22aa1n02x4               g217(.a(new_n312), .b(new_n308), .c(new_n309), .out0(new_n313));
  nor002aa1n02x5               g218(.a(new_n310), .b(new_n313), .o1(\s[29] ));
  xorb03aa1n02x5               g219(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g220(.a(new_n293), .b(new_n309), .c(new_n300), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n298), .c(new_n195), .d(new_n289), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[29] ), .b(\b[28] ), .c(new_n308), .carry(new_n318));
  xnrc02aa1n02x5               g223(.a(\b[29] ), .b(\a[30] ), .out0(new_n319));
  aoi012aa1n03x5               g224(.a(new_n319), .b(new_n317), .c(new_n318), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n316), .o1(new_n321));
  tech160nm_fiaoi012aa1n03p5x5 g226(.a(new_n321), .b(new_n292), .c(new_n290), .o1(new_n322));
  nano22aa1n03x7               g227(.a(new_n322), .b(new_n318), .c(new_n319), .out0(new_n323));
  nor002aa1n02x5               g228(.a(new_n320), .b(new_n323), .o1(\s[30] ));
  norb02aa1n02x7               g229(.a(new_n316), .b(new_n319), .out0(new_n325));
  aoai13aa1n02x7               g230(.a(new_n325), .b(new_n298), .c(new_n195), .d(new_n289), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .c(new_n318), .carry(new_n327));
  xnrc02aa1n02x5               g232(.a(\b[30] ), .b(\a[31] ), .out0(new_n328));
  aoi012aa1n03x5               g233(.a(new_n328), .b(new_n326), .c(new_n327), .o1(new_n329));
  inv000aa1n02x5               g234(.a(new_n325), .o1(new_n330));
  tech160nm_fiaoi012aa1n03p5x5 g235(.a(new_n330), .b(new_n292), .c(new_n290), .o1(new_n331));
  nano22aa1n02x5               g236(.a(new_n331), .b(new_n327), .c(new_n328), .out0(new_n332));
  nor002aa1n02x5               g237(.a(new_n329), .b(new_n332), .o1(\s[31] ));
  xnrb03aa1n02x5               g238(.a(new_n104), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi122aa1n02x5               g239(.a(new_n108), .b(new_n111), .c(new_n107), .d(new_n105), .e(new_n109), .o1(new_n335));
  aoi012aa1n02x5               g240(.a(new_n335), .b(new_n111), .c(new_n151), .o1(\s[4] ));
  xorb03aa1n02x5               g241(.a(new_n151), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norp02aa1n02x5               g242(.a(\b[4] ), .b(\a[5] ), .o1(new_n338));
  aoib12aa1n06x5               g243(.a(new_n338), .b(new_n151), .c(new_n120), .out0(new_n339));
  xnrb03aa1n02x5               g244(.a(new_n339), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fioaoi03aa1n03p5x5 g245(.a(\a[6] ), .b(\b[5] ), .c(new_n339), .o1(new_n341));
  xorb03aa1n02x5               g246(.a(new_n341), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  tech160nm_fiaoi012aa1n05x5   g247(.a(new_n116), .b(new_n341), .c(new_n117), .o1(new_n343));
  xnbna2aa1n03x5               g248(.a(new_n343), .b(new_n122), .c(new_n115), .out0(\s[8] ));
  xnbna2aa1n03x5               g249(.a(new_n131), .b(new_n153), .c(new_n129), .out0(\s[9] ));
endmodule


