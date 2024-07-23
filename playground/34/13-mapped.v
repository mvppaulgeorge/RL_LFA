// Benchmark "adder" written by ABC on Thu Jul 18 05:28:21 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n315, new_n317, new_n318, new_n320,
    new_n321, new_n322, new_n323, new_n325;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nand42aa1n08x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  orn002aa1n03x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nanp02aa1n04x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1d24x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  aob012aa1n06x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  nor042aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n06x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nor042aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand02aa1n03x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  norb02aa1n03x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nanp03aa1d12x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .o1(new_n109));
  aoi012aa1n09x5               g014(.a(new_n103), .b(new_n106), .c(new_n104), .o1(new_n110));
  nor002aa1n06x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nand22aa1n12x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand22aa1n09x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nano23aa1n06x5               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  norp02aa1n04x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand22aa1n06x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nor042aa1n04x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nand22aa1n09x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano23aa1n06x5               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  nand02aa1n02x5               g025(.a(new_n120), .b(new_n115), .o1(new_n121));
  ao0012aa1n03x7               g026(.a(new_n111), .b(new_n113), .c(new_n112), .o(new_n122));
  ao0012aa1n03x5               g027(.a(new_n116), .b(new_n118), .c(new_n117), .o(new_n123));
  aoi012aa1n09x5               g028(.a(new_n123), .b(new_n120), .c(new_n122), .o1(new_n124));
  aoai13aa1n12x5               g029(.a(new_n124), .b(new_n121), .c(new_n109), .d(new_n110), .o1(new_n125));
  nor042aa1n06x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  tech160nm_finand02aa1n03p5x5 g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n03x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n97), .c(new_n125), .d(new_n98), .o1(new_n129));
  aoi112aa1n02x5               g034(.a(new_n128), .b(new_n97), .c(new_n125), .d(new_n98), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n129), .b(new_n130), .out0(\s[10] ));
  tech160nm_fioai012aa1n04x5   g036(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n132));
  nor002aa1n06x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1n16x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n12x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n129), .c(new_n132), .out0(\s[11] ));
  norp02aa1n04x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(new_n129), .b(new_n132), .o1(new_n138));
  nanp02aa1n12x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n137), .b(new_n139), .out0(new_n140));
  aoai13aa1n03x5               g045(.a(new_n140), .b(new_n133), .c(new_n138), .d(new_n134), .o1(new_n141));
  inv000aa1d42x5               g046(.a(new_n135), .o1(new_n142));
  aoai13aa1n02x5               g047(.a(new_n139), .b(new_n142), .c(new_n129), .d(new_n132), .o1(new_n143));
  oai013aa1n02x4               g048(.a(new_n141), .b(new_n143), .c(new_n133), .d(new_n137), .o1(\s[12] ));
  nano23aa1n02x5               g049(.a(new_n97), .b(new_n137), .c(new_n139), .d(new_n98), .out0(new_n145));
  and003aa1n02x5               g050(.a(new_n145), .b(new_n135), .c(new_n128), .o(new_n146));
  nano22aa1n02x4               g051(.a(new_n137), .b(new_n134), .c(new_n139), .out0(new_n147));
  nona22aa1n03x5               g052(.a(new_n147), .b(new_n132), .c(new_n133), .out0(new_n148));
  oai012aa1n02x5               g053(.a(new_n139), .b(new_n137), .c(new_n133), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n148), .b(new_n149), .o1(new_n150));
  tech160nm_fiao0012aa1n02p5x5 g055(.a(new_n150), .b(new_n125), .c(new_n146), .o(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n12x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nor042aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand02aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand02aa1n08x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n153), .out0(new_n157));
  aoi112aa1n02x5               g062(.a(new_n154), .b(new_n157), .c(new_n151), .d(new_n155), .o1(new_n158));
  nano23aa1n03x7               g063(.a(new_n154), .b(new_n153), .c(new_n156), .d(new_n155), .out0(new_n159));
  aoai13aa1n06x5               g064(.a(new_n159), .b(new_n150), .c(new_n125), .d(new_n146), .o1(new_n160));
  tech160nm_fiaoi012aa1n04x5   g065(.a(new_n153), .b(new_n154), .c(new_n156), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(new_n160), .b(new_n161), .o1(new_n162));
  aoib12aa1n02x5               g067(.a(new_n158), .b(new_n162), .c(new_n153), .out0(\s[14] ));
  nor042aa1n03x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanp02aa1n04x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  xnbna2aa1n03x5               g071(.a(new_n166), .b(new_n160), .c(new_n161), .out0(\s[15] ));
  aobi12aa1n03x5               g072(.a(new_n166), .b(new_n160), .c(new_n161), .out0(new_n168));
  nand02aa1d16x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  nor042aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  oai022aa1n03x5               g076(.a(new_n168), .b(new_n164), .c(new_n170), .d(new_n171), .o1(new_n172));
  oai022aa1n02x5               g077(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n173));
  oai013aa1n03x4               g078(.a(new_n172), .b(new_n170), .c(new_n168), .d(new_n173), .o1(\s[16] ));
  oai012aa1n02x5               g079(.a(new_n169), .b(new_n171), .c(new_n164), .o1(new_n175));
  xnrc02aa1n03x5               g080(.a(\b[16] ), .b(\a[17] ), .out0(new_n176));
  nano23aa1n09x5               g081(.a(new_n164), .b(new_n171), .c(new_n169), .d(new_n165), .out0(new_n177));
  nanp02aa1n03x5               g082(.a(new_n177), .b(new_n159), .o1(new_n178));
  nano32aa1n03x7               g083(.a(new_n178), .b(new_n145), .c(new_n135), .d(new_n128), .out0(new_n179));
  nanp02aa1n02x5               g084(.a(new_n125), .b(new_n179), .o1(new_n180));
  aboi22aa1n03x5               g085(.a(new_n161), .b(new_n177), .c(new_n169), .d(new_n173), .out0(new_n181));
  aoai13aa1n06x5               g086(.a(new_n181), .b(new_n178), .c(new_n148), .d(new_n149), .o1(new_n182));
  aoib12aa1n02x5               g087(.a(new_n176), .b(new_n180), .c(new_n182), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n162), .b(new_n177), .o1(new_n184));
  aoi013aa1n02x4               g089(.a(new_n183), .b(new_n184), .c(new_n175), .d(new_n176), .o1(\s[17] ));
  nor042aa1n02x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  orn002aa1n24x5               g091(.a(\a[17] ), .b(\b[16] ), .o(new_n187));
  xnrc02aa1n02x5               g092(.a(\b[17] ), .b(\a[18] ), .out0(new_n188));
  nano22aa1n02x4               g093(.a(new_n183), .b(new_n187), .c(new_n188), .out0(new_n189));
  norp02aa1n02x5               g094(.a(new_n188), .b(new_n176), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n182), .c(new_n125), .d(new_n179), .o1(new_n191));
  oaoi03aa1n12x5               g096(.a(\a[18] ), .b(\b[17] ), .c(new_n187), .o1(new_n192));
  inv000aa1d42x5               g097(.a(new_n192), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(new_n191), .b(new_n193), .o1(new_n194));
  aoib12aa1n02x5               g099(.a(new_n189), .b(new_n194), .c(new_n186), .out0(\s[18] ));
  nor002aa1n16x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nand42aa1n16x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1d21x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n191), .c(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoi012aa1n02x5               g105(.a(new_n196), .b(new_n194), .c(new_n197), .o1(new_n201));
  nor042aa1n06x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n12x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n03x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  oai022aa1n02x5               g109(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n198), .o1(new_n206));
  aoai13aa1n02x5               g111(.a(new_n203), .b(new_n206), .c(new_n191), .d(new_n193), .o1(new_n207));
  oai022aa1n02x5               g112(.a(new_n201), .b(new_n204), .c(new_n207), .d(new_n205), .o1(\s[20] ));
  nano23aa1d15x5               g113(.a(new_n196), .b(new_n202), .c(new_n203), .d(new_n197), .out0(new_n209));
  nona22aa1n12x5               g114(.a(new_n209), .b(new_n188), .c(new_n176), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n182), .c(new_n125), .d(new_n179), .o1(new_n212));
  aoi112aa1n09x5               g117(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n213));
  oai112aa1n06x5               g118(.a(new_n198), .b(new_n204), .c(new_n213), .d(new_n186), .o1(new_n214));
  oai012aa1n09x5               g119(.a(new_n203), .b(new_n202), .c(new_n196), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(new_n214), .b(new_n215), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  tech160nm_fixorc02aa1n04x5   g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xnbna2aa1n03x5               g123(.a(new_n218), .b(new_n212), .c(new_n217), .out0(\s[21] ));
  nanp02aa1n03x5               g124(.a(new_n212), .b(new_n217), .o1(new_n220));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xnrc02aa1n02x5               g126(.a(\b[21] ), .b(\a[22] ), .out0(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n221), .c(new_n220), .d(new_n218), .o1(new_n223));
  oai022aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n224));
  xnrc02aa1n02x5               g129(.a(\b[20] ), .b(\a[21] ), .out0(new_n225));
  nanp02aa1n02x5               g130(.a(\b[21] ), .b(\a[22] ), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n225), .c(new_n212), .d(new_n217), .o1(new_n227));
  oai012aa1n02x5               g132(.a(new_n223), .b(new_n227), .c(new_n224), .o1(\s[22] ));
  nanb02aa1n06x5               g133(.a(new_n222), .b(new_n218), .out0(new_n229));
  norp02aa1n02x5               g134(.a(new_n210), .b(new_n229), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n182), .c(new_n125), .d(new_n179), .o1(new_n231));
  nanp02aa1n04x5               g136(.a(new_n224), .b(new_n226), .o1(new_n232));
  aoai13aa1n12x5               g137(.a(new_n232), .b(new_n229), .c(new_n214), .d(new_n215), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  xorc02aa1n12x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  xnbna2aa1n03x5               g140(.a(new_n235), .b(new_n231), .c(new_n234), .out0(\s[23] ));
  nanp02aa1n03x5               g141(.a(new_n231), .b(new_n234), .o1(new_n237));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  xnrc02aa1n02x5               g143(.a(\b[23] ), .b(\a[24] ), .out0(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n238), .c(new_n237), .d(new_n235), .o1(new_n240));
  oai022aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n235), .o1(new_n242));
  nanp02aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n242), .c(new_n231), .d(new_n234), .o1(new_n244));
  oaih12aa1n02x5               g149(.a(new_n240), .b(new_n244), .c(new_n241), .o1(\s[24] ));
  nor002aa1n02x5               g150(.a(new_n222), .b(new_n225), .o1(new_n246));
  norb02aa1n06x5               g151(.a(new_n235), .b(new_n239), .out0(new_n247));
  inv030aa1n04x5               g152(.a(new_n247), .o1(new_n248));
  nano32aa1n02x4               g153(.a(new_n248), .b(new_n246), .c(new_n190), .d(new_n209), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n182), .c(new_n125), .d(new_n179), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n215), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n246), .b(new_n251), .c(new_n209), .d(new_n192), .o1(new_n252));
  and002aa1n02x5               g157(.a(new_n241), .b(new_n243), .o(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  aoai13aa1n12x5               g159(.a(new_n254), .b(new_n248), .c(new_n252), .d(new_n232), .o1(new_n255));
  inv020aa1n02x5               g160(.a(new_n255), .o1(new_n256));
  xorc02aa1n12x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  xnbna2aa1n03x5               g162(.a(new_n257), .b(new_n250), .c(new_n256), .out0(\s[25] ));
  nanp02aa1n02x5               g163(.a(new_n250), .b(new_n256), .o1(new_n259));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xnrc02aa1n02x5               g165(.a(\b[25] ), .b(\a[26] ), .out0(new_n261));
  aoai13aa1n03x5               g166(.a(new_n261), .b(new_n260), .c(new_n259), .d(new_n257), .o1(new_n262));
  oai022aa1n02x5               g167(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n257), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n264), .c(new_n250), .d(new_n256), .o1(new_n266));
  oai012aa1n02x5               g171(.a(new_n262), .b(new_n266), .c(new_n263), .o1(\s[26] ));
  norb02aa1n06x5               g172(.a(new_n257), .b(new_n261), .out0(new_n268));
  nano23aa1n06x5               g173(.a(new_n248), .b(new_n210), .c(new_n268), .d(new_n246), .out0(new_n269));
  aoai13aa1n12x5               g174(.a(new_n269), .b(new_n182), .c(new_n125), .d(new_n179), .o1(new_n270));
  aoai13aa1n04x5               g175(.a(new_n268), .b(new_n253), .c(new_n233), .d(new_n247), .o1(new_n271));
  nanp02aa1n02x5               g176(.a(new_n263), .b(new_n265), .o1(new_n272));
  nand23aa1n06x5               g177(.a(new_n270), .b(new_n271), .c(new_n272), .o1(new_n273));
  xorb03aa1n02x5               g178(.a(new_n273), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  xorc02aa1n12x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n275), .c(new_n273), .d(new_n276), .o1(new_n278));
  oai022aa1d18x5               g183(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n279));
  aoi022aa1n12x5               g184(.a(new_n255), .b(new_n268), .c(new_n265), .d(new_n263), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n276), .o1(new_n281));
  nanp02aa1n02x5               g186(.a(\b[27] ), .b(\a[28] ), .o1(new_n282));
  aoai13aa1n04x5               g187(.a(new_n282), .b(new_n281), .c(new_n280), .d(new_n270), .o1(new_n283));
  tech160nm_fioai012aa1n03p5x5 g188(.a(new_n278), .b(new_n283), .c(new_n279), .o1(\s[28] ));
  norb02aa1n02x5               g189(.a(new_n276), .b(new_n277), .out0(new_n285));
  inv000aa1n02x5               g190(.a(new_n285), .o1(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[28] ), .b(\a[29] ), .out0(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n282), .c(new_n279), .o1(new_n288));
  aoai13aa1n02x7               g193(.a(new_n288), .b(new_n286), .c(new_n280), .d(new_n270), .o1(new_n289));
  and002aa1n02x5               g194(.a(new_n279), .b(new_n282), .o(new_n290));
  aoai13aa1n03x5               g195(.a(new_n287), .b(new_n290), .c(new_n273), .d(new_n285), .o1(new_n291));
  nanp02aa1n03x5               g196(.a(new_n291), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1d15x5               g198(.a(new_n276), .b(new_n287), .c(new_n277), .out0(new_n294));
  inv020aa1n02x5               g199(.a(new_n290), .o1(new_n295));
  oaoi03aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .o1(new_n296));
  tech160nm_fixorc02aa1n03p5x5 g201(.a(\a[30] ), .b(\b[29] ), .out0(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n296), .c(new_n273), .d(new_n294), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n294), .o1(new_n300));
  norp02aa1n02x5               g205(.a(new_n296), .b(new_n298), .o1(new_n301));
  aoai13aa1n02x7               g206(.a(new_n301), .b(new_n300), .c(new_n280), .d(new_n270), .o1(new_n302));
  nanp02aa1n03x5               g207(.a(new_n299), .b(new_n302), .o1(\s[30] ));
  nano23aa1n06x5               g208(.a(new_n287), .b(new_n277), .c(new_n297), .d(new_n276), .out0(new_n304));
  inv040aa1d30x5               g209(.a(new_n304), .o1(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  aoi012aa1n02x5               g211(.a(new_n301), .b(\a[30] ), .c(\b[29] ), .o1(new_n307));
  norp02aa1n02x5               g212(.a(new_n307), .b(new_n306), .o1(new_n308));
  aoai13aa1n02x7               g213(.a(new_n308), .b(new_n305), .c(new_n280), .d(new_n270), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n306), .b(new_n307), .c(new_n273), .d(new_n304), .o1(new_n310));
  nanp02aa1n03x5               g215(.a(new_n310), .b(new_n309), .o1(\s[31] ));
  xorb03aa1n02x5               g216(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oai012aa1n02x5               g217(.a(new_n107), .b(new_n102), .c(new_n106), .o1(new_n313));
  xnrc02aa1n02x5               g218(.a(new_n313), .b(new_n105), .out0(\s[4] ));
  norb02aa1n02x5               g219(.a(new_n114), .b(new_n113), .out0(new_n315));
  xnbna2aa1n03x5               g220(.a(new_n315), .b(new_n109), .c(new_n110), .out0(\s[5] ));
  nanp02aa1n02x5               g221(.a(new_n109), .b(new_n110), .o1(new_n317));
  aoi012aa1n02x5               g222(.a(new_n113), .b(new_n317), .c(new_n114), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g224(.a(new_n119), .b(new_n118), .out0(new_n320));
  aobi12aa1n02x5               g225(.a(new_n115), .b(new_n109), .c(new_n110), .out0(new_n321));
  oa0012aa1n02x5               g226(.a(new_n320), .b(new_n321), .c(new_n122), .o(new_n322));
  aoi112aa1n02x5               g227(.a(new_n122), .b(new_n320), .c(new_n317), .d(new_n115), .o1(new_n323));
  norp02aa1n02x5               g228(.a(new_n322), .b(new_n323), .o1(\s[7] ));
  oaoi13aa1n02x5               g229(.a(new_n118), .b(new_n119), .c(new_n321), .d(new_n122), .o1(new_n325));
  xnrb03aa1n02x5               g230(.a(new_n325), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g231(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


