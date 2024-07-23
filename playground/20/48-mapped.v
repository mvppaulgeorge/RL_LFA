// Benchmark "adder" written by ABC on Wed Jul 17 22:39:35 2024

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
    new_n119, new_n120, new_n121, new_n122, new_n123, new_n124, new_n126,
    new_n127, new_n128, new_n129, new_n131, new_n132, new_n133, new_n134,
    new_n135, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n146, new_n147, new_n148, new_n150,
    new_n151, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n160, new_n161, new_n162, new_n163, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n172, new_n173, new_n174,
    new_n175, new_n177, new_n178, new_n179, new_n180, new_n181, new_n182,
    new_n183, new_n186, new_n187, new_n188, new_n189, new_n190, new_n191,
    new_n192, new_n193, new_n195, new_n196, new_n197, new_n198, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n205, new_n206, new_n207,
    new_n208, new_n209, new_n210, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n229, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n248, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n300, new_n302, new_n305,
    new_n306, new_n307, new_n309, new_n311;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  oa0022aa1n06x5               g002(.a(\a[6] ), .b(\b[5] ), .c(\a[5] ), .d(\b[4] ), .o(new_n98));
  orn002aa1n24x5               g003(.a(\a[7] ), .b(\b[6] ), .o(new_n99));
  nand22aa1n03x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nand42aa1n06x5               g005(.a(new_n99), .b(new_n100), .o1(new_n101));
  ao0022aa1n12x5               g006(.a(\a[6] ), .b(\b[5] ), .c(\b[7] ), .d(\a[8] ), .o(new_n102));
  oao003aa1n03x5               g007(.a(\a[8] ), .b(\b[7] ), .c(new_n99), .carry(new_n103));
  oai013aa1n09x5               g008(.a(new_n103), .b(new_n101), .c(new_n102), .d(new_n98), .o1(new_n104));
  nand22aa1n12x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  nanp02aa1n04x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nor042aa1n04x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  oai112aa1n04x5               g013(.a(new_n108), .b(new_n106), .c(new_n107), .d(new_n105), .o1(new_n109));
  oa0022aa1n09x5               g014(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n110));
  aoi022aa1n12x5               g015(.a(new_n109), .b(new_n110), .c(\b[3] ), .d(\a[4] ), .o1(new_n111));
  aoi112aa1n09x5               g016(.a(new_n101), .b(new_n102), .c(\a[5] ), .d(\b[4] ), .o1(new_n112));
  nor042aa1n02x5               g017(.a(\b[8] ), .b(\a[9] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[8] ), .b(\a[9] ), .o1(new_n114));
  norb02aa1n06x4               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  aoai13aa1n06x5               g020(.a(new_n115), .b(new_n104), .c(new_n111), .d(new_n112), .o1(new_n116));
  oaib12aa1n02x5               g021(.a(new_n116), .b(\b[8] ), .c(new_n97), .out0(new_n117));
  xorb03aa1n02x5               g022(.a(new_n117), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1n10x5               g023(.a(\b[9] ), .b(\a[10] ), .o1(new_n119));
  nor002aa1n12x5               g024(.a(\b[10] ), .b(\a[11] ), .o1(new_n120));
  nanp02aa1n04x5               g025(.a(\b[10] ), .b(\a[11] ), .o1(new_n121));
  norb02aa1n02x5               g026(.a(new_n121), .b(new_n120), .out0(new_n122));
  nor042aa1n04x5               g027(.a(\b[9] ), .b(\a[10] ), .o1(new_n123));
  nona22aa1n02x4               g028(.a(new_n116), .b(new_n113), .c(new_n123), .out0(new_n124));
  xobna2aa1n03x5               g029(.a(new_n122), .b(new_n124), .c(new_n119), .out0(\s[11] ));
  inv000aa1d42x5               g030(.a(\b[11] ), .o1(new_n126));
  nanb02aa1n02x5               g031(.a(\a[12] ), .b(new_n126), .out0(new_n127));
  nand02aa1d16x5               g032(.a(\b[11] ), .b(\a[12] ), .o1(new_n128));
  aoi013aa1n03x5               g033(.a(new_n120), .b(new_n124), .c(new_n122), .d(new_n119), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n127), .c(new_n128), .out0(\s[12] ));
  aoi012aa1n06x5               g035(.a(new_n104), .b(new_n111), .c(new_n112), .o1(new_n131));
  nor042aa1n06x5               g036(.a(\b[11] ), .b(\a[12] ), .o1(new_n132));
  nano22aa1n03x7               g037(.a(new_n132), .b(new_n121), .c(new_n128), .out0(new_n133));
  norb03aa1n03x4               g038(.a(new_n119), .b(new_n123), .c(new_n120), .out0(new_n134));
  nand03aa1n02x5               g039(.a(new_n133), .b(new_n134), .c(new_n115), .o1(new_n135));
  nanb03aa1n03x5               g040(.a(new_n132), .b(new_n128), .c(new_n121), .out0(new_n136));
  inv000aa1d42x5               g041(.a(\a[11] ), .o1(new_n137));
  inv000aa1d42x5               g042(.a(\b[10] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(new_n138), .b(new_n137), .o1(new_n139));
  oai022aa1n02x5               g044(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n140));
  nand03aa1n02x5               g045(.a(new_n140), .b(new_n139), .c(new_n119), .o1(new_n141));
  aoi012aa1d18x5               g046(.a(new_n132), .b(new_n120), .c(new_n128), .o1(new_n142));
  tech160nm_fioai012aa1n05x5   g047(.a(new_n142), .b(new_n141), .c(new_n136), .o1(new_n143));
  oabi12aa1n06x5               g048(.a(new_n143), .b(new_n131), .c(new_n135), .out0(new_n144));
  xorb03aa1n02x5               g049(.a(new_n144), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n04x5               g050(.a(\b[12] ), .b(\a[13] ), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  aoi012aa1n02x5               g052(.a(new_n146), .b(new_n144), .c(new_n147), .o1(new_n148));
  xnrb03aa1n02x5               g053(.a(new_n148), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n04x5               g054(.a(\b[13] ), .b(\a[14] ), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(\b[13] ), .b(\a[14] ), .o1(new_n151));
  nano23aa1n06x5               g056(.a(new_n146), .b(new_n150), .c(new_n151), .d(new_n147), .out0(new_n152));
  oa0012aa1n09x5               g057(.a(new_n151), .b(new_n150), .c(new_n146), .o(new_n153));
  nor002aa1n20x5               g058(.a(\b[14] ), .b(\a[15] ), .o1(new_n154));
  nand42aa1d28x5               g059(.a(\b[14] ), .b(\a[15] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  aoai13aa1n06x5               g061(.a(new_n156), .b(new_n153), .c(new_n144), .d(new_n152), .o1(new_n157));
  aoi112aa1n02x5               g062(.a(new_n156), .b(new_n153), .c(new_n144), .d(new_n152), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n157), .b(new_n158), .out0(\s[15] ));
  inv000aa1d42x5               g064(.a(new_n154), .o1(new_n160));
  nor042aa1n04x5               g065(.a(\b[15] ), .b(\a[16] ), .o1(new_n161));
  nand42aa1n20x5               g066(.a(\b[15] ), .b(\a[16] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n157), .c(new_n160), .out0(\s[16] ));
  nano23aa1d15x5               g069(.a(new_n154), .b(new_n161), .c(new_n162), .d(new_n155), .out0(new_n165));
  nano22aa1n03x7               g070(.a(new_n135), .b(new_n152), .c(new_n165), .out0(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n104), .c(new_n112), .d(new_n111), .o1(new_n167));
  aoai13aa1n04x5               g072(.a(new_n165), .b(new_n153), .c(new_n143), .d(new_n152), .o1(new_n168));
  tech160nm_fiaoi012aa1n04x5   g073(.a(new_n161), .b(new_n154), .c(new_n162), .o1(new_n169));
  nand23aa1n06x5               g074(.a(new_n167), .b(new_n168), .c(new_n169), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g076(.a(\a[18] ), .o1(new_n172));
  inv000aa1d42x5               g077(.a(\a[17] ), .o1(new_n173));
  inv000aa1d42x5               g078(.a(\b[16] ), .o1(new_n174));
  oaoi03aa1n03x5               g079(.a(new_n173), .b(new_n174), .c(new_n170), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[17] ), .c(new_n172), .out0(\s[18] ));
  xroi22aa1d06x4               g081(.a(new_n173), .b(\b[16] ), .c(new_n172), .d(\b[17] ), .out0(new_n177));
  nanp02aa1n02x5               g082(.a(new_n174), .b(new_n173), .o1(new_n178));
  oaoi03aa1n02x5               g083(.a(\a[18] ), .b(\b[17] ), .c(new_n178), .o1(new_n179));
  aoi012aa1n03x5               g084(.a(new_n179), .b(new_n170), .c(new_n177), .o1(new_n180));
  nor002aa1d32x5               g085(.a(\b[18] ), .b(\a[19] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  nanp02aa1n04x5               g087(.a(\b[18] ), .b(\a[19] ), .o1(new_n183));
  xnbna2aa1n03x5               g088(.a(new_n180), .b(new_n183), .c(new_n182), .out0(\s[19] ));
  xnrc02aa1n02x5               g089(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanb02aa1n02x5               g090(.a(new_n181), .b(new_n183), .out0(new_n186));
  inv000aa1d42x5               g091(.a(new_n186), .o1(new_n187));
  aoai13aa1n03x5               g092(.a(new_n187), .b(new_n179), .c(new_n170), .d(new_n177), .o1(new_n188));
  nor002aa1n04x5               g093(.a(\b[19] ), .b(\a[20] ), .o1(new_n189));
  nand22aa1n04x5               g094(.a(\b[19] ), .b(\a[20] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(new_n189), .b(new_n190), .out0(new_n191));
  nanp03aa1n03x5               g096(.a(new_n188), .b(new_n182), .c(new_n191), .o1(new_n192));
  tech160nm_fiaoi012aa1n02p5x5 g097(.a(new_n191), .b(new_n188), .c(new_n182), .o1(new_n193));
  norb02aa1n03x4               g098(.a(new_n192), .b(new_n193), .out0(\s[20] ));
  nona23aa1n09x5               g099(.a(new_n190), .b(new_n183), .c(new_n181), .d(new_n189), .out0(new_n195));
  norb02aa1n02x5               g100(.a(new_n177), .b(new_n195), .out0(new_n196));
  oai022aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oaib12aa1n02x5               g102(.a(new_n197), .b(new_n172), .c(\b[17] ), .out0(new_n198));
  aoi012aa1n09x5               g103(.a(new_n189), .b(new_n181), .c(new_n190), .o1(new_n199));
  tech160nm_fioai012aa1n05x5   g104(.a(new_n199), .b(new_n195), .c(new_n198), .o1(new_n200));
  aoi012aa1n03x5               g105(.a(new_n200), .b(new_n170), .c(new_n196), .o1(new_n201));
  xnrc02aa1n12x5               g106(.a(\b[20] ), .b(\a[21] ), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  xnrc02aa1n02x5               g108(.a(new_n201), .b(new_n203), .out0(\s[21] ));
  nor042aa1n03x5               g109(.a(\b[20] ), .b(\a[21] ), .o1(new_n205));
  inv040aa1n03x5               g110(.a(new_n205), .o1(new_n206));
  aoai13aa1n03x5               g111(.a(new_n203), .b(new_n200), .c(new_n170), .d(new_n196), .o1(new_n207));
  xnrc02aa1n12x5               g112(.a(\b[21] ), .b(\a[22] ), .out0(new_n208));
  nanp03aa1n03x5               g113(.a(new_n207), .b(new_n206), .c(new_n208), .o1(new_n209));
  tech160nm_fiaoi012aa1n02p5x5 g114(.a(new_n208), .b(new_n207), .c(new_n206), .o1(new_n210));
  norb02aa1n03x4               g115(.a(new_n209), .b(new_n210), .out0(\s[22] ));
  inv000aa1d42x5               g116(.a(new_n153), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n165), .o1(new_n213));
  tech160nm_fioai012aa1n03p5x5 g118(.a(new_n119), .b(\b[10] ), .c(\a[11] ), .o1(new_n214));
  oab012aa1n02x4               g119(.a(new_n214), .b(new_n123), .c(new_n113), .out0(new_n215));
  inv000aa1n02x5               g120(.a(new_n142), .o1(new_n216));
  aoai13aa1n03x5               g121(.a(new_n152), .b(new_n216), .c(new_n215), .d(new_n133), .o1(new_n217));
  aoai13aa1n06x5               g122(.a(new_n169), .b(new_n213), .c(new_n217), .d(new_n212), .o1(new_n218));
  nano23aa1n06x5               g123(.a(new_n181), .b(new_n189), .c(new_n190), .d(new_n183), .out0(new_n219));
  nor042aa1n04x5               g124(.a(new_n208), .b(new_n202), .o1(new_n220));
  and003aa1n06x5               g125(.a(new_n177), .b(new_n220), .c(new_n219), .o(new_n221));
  oaib12aa1n02x7               g126(.a(new_n221), .b(new_n218), .c(new_n167), .out0(new_n222));
  oao003aa1n12x5               g127(.a(\a[22] ), .b(\b[21] ), .c(new_n206), .carry(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoi012aa1n02x5               g129(.a(new_n224), .b(new_n200), .c(new_n220), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[22] ), .b(\a[23] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n222), .c(new_n225), .out0(\s[23] ));
  nor042aa1n03x5               g133(.a(\b[22] ), .b(\a[23] ), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  inv020aa1n03x5               g135(.a(new_n225), .o1(new_n231));
  aoai13aa1n06x5               g136(.a(new_n227), .b(new_n231), .c(new_n170), .d(new_n221), .o1(new_n232));
  xnrc02aa1n02x5               g137(.a(\b[23] ), .b(\a[24] ), .out0(new_n233));
  nand43aa1n02x5               g138(.a(new_n232), .b(new_n230), .c(new_n233), .o1(new_n234));
  tech160nm_fiaoi012aa1n02p5x5 g139(.a(new_n233), .b(new_n232), .c(new_n230), .o1(new_n235));
  norb02aa1n03x4               g140(.a(new_n234), .b(new_n235), .out0(\s[24] ));
  nor002aa1n02x5               g141(.a(new_n233), .b(new_n226), .o1(new_n237));
  inv000aa1n02x5               g142(.a(new_n237), .o1(new_n238));
  nano32aa1n02x4               g143(.a(new_n238), .b(new_n177), .c(new_n220), .d(new_n219), .out0(new_n239));
  inv000aa1n02x5               g144(.a(new_n199), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n220), .b(new_n240), .c(new_n219), .d(new_n179), .o1(new_n241));
  oao003aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .c(new_n230), .carry(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n238), .c(new_n241), .d(new_n223), .o1(new_n243));
  aoi012aa1n03x5               g148(.a(new_n243), .b(new_n170), .c(new_n239), .o1(new_n244));
  xnrc02aa1n12x5               g149(.a(\b[24] ), .b(\a[25] ), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  xnrc02aa1n02x5               g151(.a(new_n244), .b(new_n246), .out0(\s[25] ));
  nor042aa1n03x5               g152(.a(\b[24] ), .b(\a[25] ), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  aoai13aa1n03x5               g154(.a(new_n246), .b(new_n243), .c(new_n170), .d(new_n239), .o1(new_n250));
  xnrc02aa1n02x5               g155(.a(\b[25] ), .b(\a[26] ), .out0(new_n251));
  nand43aa1n02x5               g156(.a(new_n250), .b(new_n249), .c(new_n251), .o1(new_n252));
  tech160nm_fiaoi012aa1n02p5x5 g157(.a(new_n251), .b(new_n250), .c(new_n249), .o1(new_n253));
  norb02aa1n03x4               g158(.a(new_n252), .b(new_n253), .out0(\s[26] ));
  nor042aa1n02x5               g159(.a(new_n251), .b(new_n245), .o1(new_n255));
  and003aa1n06x5               g160(.a(new_n221), .b(new_n255), .c(new_n237), .o(new_n256));
  oaib12aa1n09x5               g161(.a(new_n256), .b(new_n218), .c(new_n167), .out0(new_n257));
  oao003aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .c(new_n249), .carry(new_n258));
  aobi12aa1n06x5               g163(.a(new_n258), .b(new_n243), .c(new_n255), .out0(new_n259));
  xorc02aa1n12x5               g164(.a(\a[27] ), .b(\b[26] ), .out0(new_n260));
  xnbna2aa1n06x5               g165(.a(new_n260), .b(new_n259), .c(new_n257), .out0(\s[27] ));
  norp02aa1n02x5               g166(.a(\b[26] ), .b(\a[27] ), .o1(new_n262));
  inv040aa1n03x5               g167(.a(new_n262), .o1(new_n263));
  aobi12aa1n06x5               g168(.a(new_n260), .b(new_n259), .c(new_n257), .out0(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[27] ), .b(\a[28] ), .out0(new_n265));
  nano22aa1n03x5               g170(.a(new_n264), .b(new_n263), .c(new_n265), .out0(new_n266));
  inv000aa1n02x5               g171(.a(new_n255), .o1(new_n267));
  nona32aa1n03x5               g172(.a(new_n221), .b(new_n267), .c(new_n233), .d(new_n226), .out0(new_n268));
  aoi013aa1n06x4               g173(.a(new_n268), .b(new_n167), .c(new_n168), .d(new_n169), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n237), .b(new_n224), .c(new_n200), .d(new_n220), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n258), .b(new_n267), .c(new_n270), .d(new_n242), .o1(new_n271));
  oaih12aa1n02x5               g176(.a(new_n260), .b(new_n271), .c(new_n269), .o1(new_n272));
  tech160nm_fiaoi012aa1n02p5x5 g177(.a(new_n265), .b(new_n272), .c(new_n263), .o1(new_n273));
  norp02aa1n03x5               g178(.a(new_n273), .b(new_n266), .o1(\s[28] ));
  norb02aa1n02x5               g179(.a(new_n260), .b(new_n265), .out0(new_n275));
  oaih12aa1n02x5               g180(.a(new_n275), .b(new_n271), .c(new_n269), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[28] ), .b(\b[27] ), .c(new_n263), .carry(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[28] ), .b(\a[29] ), .out0(new_n278));
  tech160nm_fiaoi012aa1n02p5x5 g183(.a(new_n278), .b(new_n276), .c(new_n277), .o1(new_n279));
  aobi12aa1n06x5               g184(.a(new_n275), .b(new_n259), .c(new_n257), .out0(new_n280));
  nano22aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n278), .out0(new_n281));
  norp02aa1n03x5               g186(.a(new_n279), .b(new_n281), .o1(\s[29] ));
  xorb03aa1n02x5               g187(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g188(.a(new_n260), .b(new_n278), .c(new_n265), .out0(new_n284));
  oaih12aa1n02x5               g189(.a(new_n284), .b(new_n271), .c(new_n269), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[29] ), .b(\b[28] ), .c(new_n277), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[29] ), .b(\a[30] ), .out0(new_n287));
  tech160nm_fiaoi012aa1n02p5x5 g192(.a(new_n287), .b(new_n285), .c(new_n286), .o1(new_n288));
  aobi12aa1n06x5               g193(.a(new_n284), .b(new_n259), .c(new_n257), .out0(new_n289));
  nano22aa1n03x5               g194(.a(new_n289), .b(new_n286), .c(new_n287), .out0(new_n290));
  norp02aa1n03x5               g195(.a(new_n288), .b(new_n290), .o1(\s[30] ));
  norb02aa1n02x5               g196(.a(new_n284), .b(new_n287), .out0(new_n292));
  oai012aa1n02x5               g197(.a(new_n292), .b(new_n271), .c(new_n269), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[30] ), .b(\b[29] ), .c(new_n286), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[30] ), .b(\a[31] ), .out0(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n06x5               g201(.a(new_n292), .b(new_n259), .c(new_n257), .out0(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[31] ));
  oai012aa1n02x5               g204(.a(new_n106), .b(new_n107), .c(new_n105), .o1(new_n300));
  xnrb03aa1n02x5               g205(.a(new_n300), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g206(.a(\a[3] ), .b(\b[2] ), .c(new_n300), .o1(new_n302));
  xorb03aa1n02x5               g207(.a(new_n302), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g208(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g209(.a(\a[5] ), .o1(new_n305));
  inv000aa1d42x5               g210(.a(\b[4] ), .o1(new_n306));
  oaoi03aa1n02x5               g211(.a(new_n305), .b(new_n306), .c(new_n111), .o1(new_n307));
  xnrb03aa1n02x5               g212(.a(new_n307), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g213(.a(\a[6] ), .b(\b[5] ), .c(new_n307), .carry(new_n309));
  xnbna2aa1n03x5               g214(.a(new_n309), .b(new_n99), .c(new_n100), .out0(\s[7] ));
  oaoi03aa1n02x5               g215(.a(\a[7] ), .b(\b[6] ), .c(new_n309), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g217(.a(new_n131), .b(\b[8] ), .c(new_n97), .out0(\s[9] ));
endmodule


