// Benchmark "adder" written by ABC on Wed Jul 17 17:41:50 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n340, new_n343, new_n345, new_n347;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  and002aa1n02x7               g002(.a(\b[3] ), .b(\a[4] ), .o(new_n98));
  inv000aa1n02x5               g003(.a(new_n98), .o1(new_n99));
  nor042aa1n06x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  norb02aa1n02x5               g006(.a(new_n101), .b(new_n100), .out0(new_n102));
  nor042aa1n06x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  inv000aa1d42x5               g008(.a(new_n103), .o1(new_n104));
  nand02aa1n03x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(new_n107), .b(new_n104), .o1(new_n108));
  oab012aa1d24x5               g013(.a(new_n100), .b(\a[4] ), .c(\b[3] ), .out0(new_n109));
  inv000aa1d42x5               g014(.a(new_n109), .o1(new_n110));
  aoai13aa1n06x5               g015(.a(new_n99), .b(new_n110), .c(new_n108), .d(new_n102), .o1(new_n111));
  norp02aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand02aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n03x7               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  norp02aa1n04x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nand02aa1n04x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nor042aa1n03x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nand42aa1n04x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nano23aa1n03x7               g025(.a(new_n117), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  aoi012aa1n02x7               g027(.a(new_n117), .b(new_n119), .c(new_n118), .o1(new_n123));
  inv000aa1d42x5               g028(.a(new_n114), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .o1(new_n125));
  aoib12aa1n09x5               g030(.a(new_n125), .b(new_n116), .c(new_n123), .out0(new_n126));
  oai012aa1n06x5               g031(.a(new_n126), .b(new_n111), .c(new_n122), .o1(new_n127));
  nand42aa1n03x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  aoi012aa1n02x5               g033(.a(new_n97), .b(new_n127), .c(new_n128), .o1(new_n129));
  xnrb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n12x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand02aa1d28x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nano23aa1n09x5               g037(.a(new_n131), .b(new_n97), .c(new_n128), .d(new_n132), .out0(new_n133));
  aoi012aa1d24x5               g038(.a(new_n131), .b(new_n97), .c(new_n132), .o1(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  aoi012aa1n03x5               g040(.a(new_n135), .b(new_n127), .c(new_n133), .o1(new_n136));
  xnrb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n03x5               g042(.a(\a[11] ), .b(\b[10] ), .c(new_n136), .o1(new_n138));
  xorb03aa1n02x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  inv000aa1d42x5               g044(.a(\b[2] ), .o1(new_n140));
  nanb02aa1n02x5               g045(.a(\a[3] ), .b(new_n140), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n141), .b(new_n101), .o1(new_n142));
  aoai13aa1n04x5               g047(.a(new_n109), .b(new_n142), .c(new_n107), .d(new_n104), .o1(new_n143));
  nona23aa1n06x5               g048(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n144));
  nona23aa1n09x5               g049(.a(new_n143), .b(new_n121), .c(new_n144), .d(new_n98), .out0(new_n145));
  nor042aa1n06x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  nand22aa1n04x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  nor002aa1n12x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nand02aa1n04x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nano23aa1d15x5               g054(.a(new_n146), .b(new_n148), .c(new_n149), .d(new_n147), .out0(new_n150));
  tech160nm_fiaoi012aa1n03p5x5 g055(.a(new_n148), .b(new_n146), .c(new_n149), .o1(new_n151));
  aobi12aa1n12x5               g056(.a(new_n151), .b(new_n150), .c(new_n135), .out0(new_n152));
  nand22aa1n12x5               g057(.a(new_n150), .b(new_n133), .o1(new_n153));
  aoai13aa1n03x5               g058(.a(new_n152), .b(new_n153), .c(new_n145), .d(new_n126), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1d18x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand02aa1n04x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  tech160nm_fiaoi012aa1n04x5   g064(.a(new_n103), .b(new_n105), .c(new_n106), .o1(new_n160));
  oaoi13aa1n04x5               g065(.a(new_n98), .b(new_n109), .c(new_n160), .d(new_n142), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n118), .b(new_n117), .out0(new_n162));
  norb02aa1n02x7               g067(.a(new_n120), .b(new_n119), .out0(new_n163));
  nano22aa1n03x7               g068(.a(new_n144), .b(new_n162), .c(new_n163), .out0(new_n164));
  oabi12aa1n02x5               g069(.a(new_n125), .b(new_n144), .c(new_n123), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n153), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n165), .c(new_n161), .d(new_n164), .o1(new_n167));
  norp02aa1n24x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand02aa1d28x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  aoi012aa1d24x5               g074(.a(new_n168), .b(new_n156), .c(new_n169), .o1(new_n170));
  nona23aa1n09x5               g075(.a(new_n169), .b(new_n157), .c(new_n156), .d(new_n168), .out0(new_n171));
  aoai13aa1n04x5               g076(.a(new_n170), .b(new_n171), .c(new_n167), .d(new_n152), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nanp02aa1n04x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanb02aa1n09x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  norp02aa1n12x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand42aa1n08x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanb02aa1n06x5               g084(.a(new_n178), .b(new_n179), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(new_n181), .b(new_n174), .c(new_n172), .d(new_n177), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n174), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n170), .o1(new_n184));
  nano23aa1n06x5               g089(.a(new_n156), .b(new_n168), .c(new_n169), .d(new_n157), .out0(new_n185));
  aoai13aa1n03x5               g090(.a(new_n177), .b(new_n184), .c(new_n154), .d(new_n185), .o1(new_n186));
  aoi012aa1n02x5               g091(.a(new_n180), .b(new_n186), .c(new_n183), .o1(new_n187));
  norp02aa1n02x5               g092(.a(new_n187), .b(new_n182), .o1(\s[16] ));
  nona22aa1n09x5               g093(.a(new_n185), .b(new_n180), .c(new_n176), .out0(new_n189));
  nor042aa1n06x5               g094(.a(new_n189), .b(new_n153), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n165), .c(new_n161), .d(new_n164), .o1(new_n191));
  nona23aa1n09x5               g096(.a(new_n149), .b(new_n147), .c(new_n146), .d(new_n148), .out0(new_n192));
  oai012aa1n06x5               g097(.a(new_n151), .b(new_n192), .c(new_n134), .o1(new_n193));
  nona23aa1n09x5               g098(.a(new_n179), .b(new_n175), .c(new_n174), .d(new_n178), .out0(new_n194));
  nor002aa1n03x5               g099(.a(new_n194), .b(new_n171), .o1(new_n195));
  aoi012aa1n02x7               g100(.a(new_n178), .b(new_n174), .c(new_n179), .o1(new_n196));
  oai012aa1n06x5               g101(.a(new_n196), .b(new_n194), .c(new_n170), .o1(new_n197));
  aoi012aa1d18x5               g102(.a(new_n197), .b(new_n193), .c(new_n195), .o1(new_n198));
  tech160nm_fixnrc02aa1n04x5   g103(.a(\b[16] ), .b(\a[17] ), .out0(new_n199));
  xobna2aa1n03x5               g104(.a(new_n199), .b(new_n191), .c(new_n198), .out0(\s[17] ));
  norp02aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  tech160nm_fixnrc02aa1n04x5   g107(.a(\b[17] ), .b(\a[18] ), .out0(new_n203));
  nona23aa1n09x5               g108(.a(new_n185), .b(new_n133), .c(new_n194), .d(new_n192), .out0(new_n204));
  aoai13aa1n06x5               g109(.a(new_n198), .b(new_n204), .c(new_n145), .d(new_n126), .o1(new_n205));
  aob012aa1n06x5               g110(.a(new_n205), .b(\b[16] ), .c(\a[17] ), .out0(new_n206));
  xobna2aa1n03x5               g111(.a(new_n203), .b(new_n206), .c(new_n202), .out0(\s[18] ));
  nor042aa1n06x5               g112(.a(new_n203), .b(new_n199), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  nor042aa1n04x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  aoi112aa1n09x5               g115(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n211));
  nor042aa1n09x5               g116(.a(new_n211), .b(new_n210), .o1(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n209), .c(new_n191), .d(new_n198), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n20x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  nand02aa1n04x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nor002aa1d24x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand02aa1d12x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1d27x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  aoi112aa1n03x4               g125(.a(new_n216), .b(new_n220), .c(new_n213), .d(new_n217), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n216), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n212), .o1(new_n223));
  norb02aa1n03x5               g128(.a(new_n217), .b(new_n216), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n223), .c(new_n205), .d(new_n208), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n220), .o1(new_n226));
  aoi012aa1n03x5               g131(.a(new_n226), .b(new_n225), .c(new_n222), .o1(new_n227));
  norp02aa1n03x5               g132(.a(new_n227), .b(new_n221), .o1(\s[20] ));
  nona23aa1d18x5               g133(.a(new_n219), .b(new_n217), .c(new_n216), .d(new_n218), .out0(new_n229));
  inv040aa1n02x5               g134(.a(new_n229), .o1(new_n230));
  nand22aa1n12x5               g135(.a(new_n208), .b(new_n230), .o1(new_n231));
  aoi012aa1n12x5               g136(.a(new_n218), .b(new_n216), .c(new_n219), .o1(new_n232));
  oai012aa1d24x5               g137(.a(new_n232), .b(new_n229), .c(new_n212), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoai13aa1n04x5               g139(.a(new_n234), .b(new_n231), .c(new_n191), .d(new_n198), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[21] ), .b(\b[20] ), .out0(new_n238));
  xorc02aa1n12x5               g143(.a(\a[22] ), .b(\b[21] ), .out0(new_n239));
  aoi112aa1n03x4               g144(.a(new_n237), .b(new_n239), .c(new_n235), .d(new_n238), .o1(new_n240));
  inv000aa1n06x5               g145(.a(new_n237), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n231), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n238), .b(new_n233), .c(new_n205), .d(new_n242), .o1(new_n243));
  xnrc02aa1n02x5               g148(.a(\b[21] ), .b(\a[22] ), .out0(new_n244));
  aoi012aa1n03x5               g149(.a(new_n244), .b(new_n243), .c(new_n241), .o1(new_n245));
  nor002aa1n02x5               g150(.a(new_n245), .b(new_n240), .o1(\s[22] ));
  nanp02aa1n04x5               g151(.a(new_n239), .b(new_n238), .o1(new_n247));
  nano22aa1n02x4               g152(.a(new_n247), .b(new_n208), .c(new_n230), .out0(new_n248));
  inv000aa1n02x5               g153(.a(new_n248), .o1(new_n249));
  inv040aa1n02x5               g154(.a(new_n247), .o1(new_n250));
  oaoi03aa1n06x5               g155(.a(\a[22] ), .b(\b[21] ), .c(new_n241), .o1(new_n251));
  aoi012aa1d18x5               g156(.a(new_n251), .b(new_n233), .c(new_n250), .o1(new_n252));
  aoai13aa1n04x5               g157(.a(new_n252), .b(new_n249), .c(new_n191), .d(new_n198), .o1(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1d18x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  nand02aa1d04x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(new_n257));
  nor042aa1n06x5               g162(.a(\b[23] ), .b(\a[24] ), .o1(new_n258));
  nand02aa1d04x5               g163(.a(\b[23] ), .b(\a[24] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  aoi112aa1n03x4               g165(.a(new_n255), .b(new_n260), .c(new_n253), .d(new_n257), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n255), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n252), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n257), .b(new_n263), .c(new_n205), .d(new_n248), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n260), .o1(new_n265));
  tech160nm_fiaoi012aa1n02p5x5 g170(.a(new_n265), .b(new_n264), .c(new_n262), .o1(new_n266));
  nor042aa1n03x5               g171(.a(new_n266), .b(new_n261), .o1(\s[24] ));
  nona23aa1d18x5               g172(.a(new_n259), .b(new_n256), .c(new_n255), .d(new_n258), .out0(new_n268));
  inv040aa1n02x5               g173(.a(new_n268), .o1(new_n269));
  nano32aa1n03x7               g174(.a(new_n231), .b(new_n269), .c(new_n238), .d(new_n239), .out0(new_n270));
  inv000aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n258), .b(new_n255), .c(new_n259), .o1(new_n272));
  oaib12aa1n09x5               g177(.a(new_n272), .b(new_n268), .c(new_n251), .out0(new_n273));
  aoi013aa1n09x5               g178(.a(new_n273), .b(new_n233), .c(new_n250), .d(new_n269), .o1(new_n274));
  aoai13aa1n04x5               g179(.a(new_n274), .b(new_n271), .c(new_n191), .d(new_n198), .o1(new_n275));
  xorb03aa1n02x5               g180(.a(new_n275), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g181(.a(\b[24] ), .b(\a[25] ), .o1(new_n277));
  tech160nm_fixorc02aa1n03p5x5 g182(.a(\a[25] ), .b(\b[24] ), .out0(new_n278));
  xorc02aa1n12x5               g183(.a(\a[26] ), .b(\b[25] ), .out0(new_n279));
  aoi112aa1n03x4               g184(.a(new_n277), .b(new_n279), .c(new_n275), .d(new_n278), .o1(new_n280));
  inv040aa1n03x5               g185(.a(new_n277), .o1(new_n281));
  inv040aa1n03x5               g186(.a(new_n274), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n278), .b(new_n282), .c(new_n205), .d(new_n270), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n279), .o1(new_n284));
  aoi012aa1n03x5               g189(.a(new_n284), .b(new_n283), .c(new_n281), .o1(new_n285));
  nor002aa1n02x5               g190(.a(new_n285), .b(new_n280), .o1(\s[26] ));
  oabi12aa1n03x5               g191(.a(new_n197), .b(new_n152), .c(new_n189), .out0(new_n287));
  and002aa1n12x5               g192(.a(new_n279), .b(new_n278), .o(new_n288));
  nano32aa1n03x7               g193(.a(new_n231), .b(new_n288), .c(new_n250), .d(new_n269), .out0(new_n289));
  aoai13aa1n06x5               g194(.a(new_n289), .b(new_n287), .c(new_n127), .d(new_n190), .o1(new_n290));
  oai112aa1n03x5               g195(.a(new_n224), .b(new_n220), .c(new_n211), .d(new_n210), .o1(new_n291));
  aoi112aa1n02x5               g196(.a(new_n268), .b(new_n247), .c(new_n291), .d(new_n232), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[26] ), .b(\b[25] ), .c(new_n281), .carry(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  oaoi13aa1n09x5               g199(.a(new_n294), .b(new_n288), .c(new_n292), .d(new_n273), .o1(new_n295));
  norp02aa1n02x5               g200(.a(\b[26] ), .b(\a[27] ), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  norb02aa1n02x5               g202(.a(new_n297), .b(new_n296), .out0(new_n298));
  xnbna2aa1n03x5               g203(.a(new_n298), .b(new_n290), .c(new_n295), .out0(\s[27] ));
  inv020aa1n02x5               g204(.a(new_n296), .o1(new_n300));
  xnrc02aa1n12x5               g205(.a(\b[27] ), .b(\a[28] ), .out0(new_n301));
  nona22aa1n03x5               g206(.a(new_n233), .b(new_n247), .c(new_n268), .out0(new_n302));
  aobi12aa1n02x5               g207(.a(new_n272), .b(new_n269), .c(new_n251), .out0(new_n303));
  inv000aa1d42x5               g208(.a(new_n288), .o1(new_n304));
  aoai13aa1n06x5               g209(.a(new_n293), .b(new_n304), .c(new_n302), .d(new_n303), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n297), .b(new_n305), .c(new_n205), .d(new_n289), .o1(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n301), .b(new_n306), .c(new_n300), .o1(new_n307));
  aoi022aa1n02x7               g212(.a(new_n290), .b(new_n295), .c(\b[26] ), .d(\a[27] ), .o1(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n300), .c(new_n301), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[28] ));
  nano22aa1n12x5               g215(.a(new_n301), .b(new_n300), .c(new_n297), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n305), .c(new_n205), .d(new_n289), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[28] ), .b(\b[27] ), .c(new_n300), .carry(new_n313));
  xnrc02aa1n12x5               g218(.a(\b[28] ), .b(\a[29] ), .out0(new_n314));
  tech160nm_fiaoi012aa1n02p5x5 g219(.a(new_n314), .b(new_n312), .c(new_n313), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n311), .o1(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n316), .b(new_n290), .c(new_n295), .o1(new_n317));
  nano22aa1n03x5               g222(.a(new_n317), .b(new_n313), .c(new_n314), .out0(new_n318));
  nor002aa1n02x5               g223(.a(new_n315), .b(new_n318), .o1(\s[29] ));
  xorb03aa1n02x5               g224(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g225(.a(new_n298), .b(new_n314), .c(new_n301), .out0(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n305), .c(new_n205), .d(new_n289), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[29] ), .b(\b[28] ), .c(new_n313), .carry(new_n323));
  xnrc02aa1n02x5               g228(.a(\b[29] ), .b(\a[30] ), .out0(new_n324));
  tech160nm_fiaoi012aa1n02p5x5 g229(.a(new_n324), .b(new_n322), .c(new_n323), .o1(new_n325));
  inv000aa1d42x5               g230(.a(new_n321), .o1(new_n326));
  tech160nm_fiaoi012aa1n02p5x5 g231(.a(new_n326), .b(new_n290), .c(new_n295), .o1(new_n327));
  nano22aa1n03x5               g232(.a(new_n327), .b(new_n323), .c(new_n324), .out0(new_n328));
  norp02aa1n03x5               g233(.a(new_n325), .b(new_n328), .o1(\s[30] ));
  xnrc02aa1n02x5               g234(.a(\b[30] ), .b(\a[31] ), .out0(new_n330));
  norb03aa1n12x5               g235(.a(new_n311), .b(new_n324), .c(new_n314), .out0(new_n331));
  inv000aa1d42x5               g236(.a(new_n331), .o1(new_n332));
  tech160nm_fiaoi012aa1n02p5x5 g237(.a(new_n332), .b(new_n290), .c(new_n295), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n323), .carry(new_n334));
  nano22aa1n03x5               g239(.a(new_n333), .b(new_n330), .c(new_n334), .out0(new_n335));
  aoai13aa1n03x5               g240(.a(new_n331), .b(new_n305), .c(new_n205), .d(new_n289), .o1(new_n336));
  tech160nm_fiaoi012aa1n02p5x5 g241(.a(new_n330), .b(new_n336), .c(new_n334), .o1(new_n337));
  norp02aa1n03x5               g242(.a(new_n337), .b(new_n335), .o1(\s[31] ));
  xnbna2aa1n03x5               g243(.a(new_n160), .b(new_n141), .c(new_n101), .out0(\s[3] ));
  oaoi03aa1n02x5               g244(.a(\a[3] ), .b(\b[2] ), .c(new_n160), .o1(new_n340));
  xorb03aa1n02x5               g245(.a(new_n340), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xobna2aa1n03x5               g246(.a(new_n163), .b(new_n143), .c(new_n99), .out0(\s[5] ));
  nanp02aa1n02x5               g247(.a(new_n111), .b(new_n163), .o1(new_n343));
  xobna2aa1n03x5               g248(.a(new_n162), .b(new_n343), .c(new_n120), .out0(\s[6] ));
  aoi013aa1n06x4               g249(.a(new_n117), .b(new_n343), .c(new_n120), .d(new_n118), .o1(new_n345));
  xnbna2aa1n03x5               g250(.a(new_n345), .b(new_n124), .c(new_n115), .out0(\s[7] ));
  oaoi03aa1n02x5               g251(.a(\a[7] ), .b(\b[6] ), .c(new_n345), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g253(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


