// Benchmark "adder" written by ABC on Thu Jul 18 05:18:32 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n210, new_n211, new_n212, new_n213, new_n214,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n245, new_n246,
    new_n247, new_n248, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n308, new_n309, new_n311;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n03x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1n20x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nand02aa1d24x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n12x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  tech160nm_fixnrc02aa1n02p5x5 g005(.a(\b[2] ), .b(\a[3] ), .out0(new_n101));
  inv000aa1d42x5               g006(.a(\a[3] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\a[4] ), .o1(new_n103));
  inv000aa1d42x5               g008(.a(\b[3] ), .o1(new_n104));
  aboi22aa1d24x5               g009(.a(\b[2] ), .b(new_n102), .c(new_n104), .d(new_n103), .out0(new_n105));
  oai012aa1n09x5               g010(.a(new_n105), .b(new_n101), .c(new_n100), .o1(new_n106));
  nand42aa1n06x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  nand02aa1n08x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nor022aa1n16x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nanb02aa1n06x5               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  nor002aa1d32x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  inv040aa1n02x5               g016(.a(new_n111), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  oai112aa1n06x5               g018(.a(new_n112), .b(new_n113), .c(new_n104), .d(new_n103), .o1(new_n114));
  nand02aa1d08x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nor002aa1n06x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nor002aa1n03x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nona23aa1n09x5               g022(.a(new_n107), .b(new_n115), .c(new_n117), .d(new_n116), .out0(new_n118));
  nor043aa1n06x5               g023(.a(new_n118), .b(new_n114), .c(new_n110), .o1(new_n119));
  oai112aa1n06x5               g024(.a(new_n108), .b(new_n115), .c(new_n111), .d(new_n109), .o1(new_n120));
  nona23aa1n09x5               g025(.a(new_n120), .b(new_n107), .c(new_n116), .d(new_n117), .out0(new_n121));
  aoi022aa1n12x5               g026(.a(new_n119), .b(new_n106), .c(new_n121), .d(new_n107), .o1(new_n122));
  tech160nm_fioaoi03aa1n03p5x5 g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand22aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  xorc02aa1n02x5               g031(.a(\a[11] ), .b(\b[10] ), .out0(new_n127));
  oaoi13aa1n02x5               g032(.a(new_n127), .b(new_n126), .c(new_n123), .d(new_n125), .o1(new_n128));
  oai112aa1n04x5               g033(.a(new_n127), .b(new_n126), .c(new_n123), .d(new_n125), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(\s[11] ));
  orn002aa1n24x5               g035(.a(\a[11] ), .b(\b[10] ), .o(new_n131));
  xnrc02aa1n02x5               g036(.a(\b[11] ), .b(\a[12] ), .out0(new_n132));
  aoi012aa1n03x5               g037(.a(new_n132), .b(new_n129), .c(new_n131), .o1(new_n133));
  nand23aa1n03x5               g038(.a(new_n129), .b(new_n131), .c(new_n132), .o1(new_n134));
  norb02aa1n02x7               g039(.a(new_n134), .b(new_n133), .out0(\s[12] ));
  norb02aa1n03x5               g040(.a(new_n126), .b(new_n125), .out0(new_n136));
  xorc02aa1n12x5               g041(.a(\a[9] ), .b(\b[8] ), .out0(new_n137));
  and002aa1n03x5               g042(.a(\b[10] ), .b(\a[11] ), .o(new_n138));
  orn002aa1n24x5               g043(.a(\a[12] ), .b(\b[11] ), .o(new_n139));
  and002aa1n12x5               g044(.a(\b[11] ), .b(\a[12] ), .o(new_n140));
  nona23aa1d18x5               g045(.a(new_n139), .b(new_n131), .c(new_n140), .d(new_n138), .out0(new_n141));
  nano22aa1d15x5               g046(.a(new_n141), .b(new_n136), .c(new_n137), .out0(new_n142));
  inv000aa1d42x5               g047(.a(new_n142), .o1(new_n143));
  nor002aa1n03x5               g048(.a(\b[8] ), .b(\a[9] ), .o1(new_n144));
  aoi022aa1d24x5               g049(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n145));
  tech160nm_fioai012aa1n03p5x5 g050(.a(new_n145), .b(new_n144), .c(new_n125), .o1(new_n146));
  aoi013aa1n06x4               g051(.a(new_n140), .b(new_n146), .c(new_n131), .d(new_n139), .o1(new_n147));
  oabi12aa1n06x5               g052(.a(new_n147), .b(new_n122), .c(new_n143), .out0(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nanp02aa1n04x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  aoi012aa1n02x5               g056(.a(new_n150), .b(new_n148), .c(new_n151), .o1(new_n152));
  xnrb03aa1n02x5               g057(.a(new_n152), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nand02aa1n12x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nona23aa1d16x5               g060(.a(new_n155), .b(new_n151), .c(new_n150), .d(new_n154), .out0(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  oai012aa1n12x5               g062(.a(new_n155), .b(new_n154), .c(new_n150), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  xnrc02aa1n12x5               g064(.a(\b[14] ), .b(\a[15] ), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  aoai13aa1n06x5               g066(.a(new_n161), .b(new_n159), .c(new_n148), .d(new_n157), .o1(new_n162));
  aoi112aa1n02x5               g067(.a(new_n161), .b(new_n159), .c(new_n148), .d(new_n157), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(\s[15] ));
  orn002aa1n02x5               g069(.a(\a[15] ), .b(\b[14] ), .o(new_n165));
  xnrc02aa1n06x5               g070(.a(\b[15] ), .b(\a[16] ), .out0(new_n166));
  aoi012aa1n02x5               g071(.a(new_n166), .b(new_n162), .c(new_n165), .o1(new_n167));
  nanp03aa1n02x5               g072(.a(new_n162), .b(new_n165), .c(new_n166), .o1(new_n168));
  norb02aa1n02x7               g073(.a(new_n168), .b(new_n167), .out0(\s[16] ));
  nor043aa1d12x5               g074(.a(new_n156), .b(new_n160), .c(new_n166), .o1(new_n170));
  nand22aa1n09x5               g075(.a(new_n142), .b(new_n170), .o1(new_n171));
  norp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  ao0022aa1n03x5               g077(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o(new_n173));
  tech160nm_fiaoi012aa1n04x5   g078(.a(new_n173), .b(new_n158), .c(new_n165), .o1(new_n174));
  aoi112aa1n09x5               g079(.a(new_n174), .b(new_n172), .c(new_n147), .d(new_n170), .o1(new_n175));
  oai012aa1d24x5               g080(.a(new_n175), .b(new_n122), .c(new_n171), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g082(.a(\a[18] ), .o1(new_n178));
  inv040aa1d32x5               g083(.a(\a[17] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[16] ), .o1(new_n180));
  oaoi03aa1n02x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(new_n178), .out0(\s[18] ));
  xroi22aa1d06x4               g087(.a(new_n179), .b(\b[16] ), .c(new_n178), .d(\b[17] ), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n180), .b(new_n179), .o1(new_n184));
  oaoi03aa1n12x5               g089(.a(\a[18] ), .b(\b[17] ), .c(new_n184), .o1(new_n185));
  nor002aa1d32x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  nanp02aa1n04x5               g091(.a(\b[18] ), .b(\a[19] ), .o1(new_n187));
  norb02aa1n02x5               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n185), .c(new_n176), .d(new_n183), .o1(new_n189));
  aoi112aa1n02x5               g094(.a(new_n188), .b(new_n185), .c(new_n176), .d(new_n183), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n189), .b(new_n190), .out0(\s[19] ));
  xnrc02aa1n02x5               g096(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  nand02aa1n08x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  nona22aa1n03x5               g100(.a(new_n189), .b(new_n195), .c(new_n186), .out0(new_n196));
  inv000aa1d42x5               g101(.a(new_n186), .o1(new_n197));
  aobi12aa1n06x5               g102(.a(new_n195), .b(new_n189), .c(new_n197), .out0(new_n198));
  norb02aa1n03x4               g103(.a(new_n196), .b(new_n198), .out0(\s[20] ));
  nona23aa1n09x5               g104(.a(new_n194), .b(new_n187), .c(new_n186), .d(new_n193), .out0(new_n200));
  norb02aa1n06x5               g105(.a(new_n183), .b(new_n200), .out0(new_n201));
  oai022aa1n02x5               g106(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n202));
  oaib12aa1n02x5               g107(.a(new_n202), .b(new_n178), .c(\b[17] ), .out0(new_n203));
  aoi012aa1d18x5               g108(.a(new_n193), .b(new_n186), .c(new_n194), .o1(new_n204));
  oai012aa1n12x5               g109(.a(new_n204), .b(new_n200), .c(new_n203), .o1(new_n205));
  tech160nm_fixorc02aa1n03p5x5 g110(.a(\a[21] ), .b(\b[20] ), .out0(new_n206));
  aoai13aa1n12x5               g111(.a(new_n206), .b(new_n205), .c(new_n176), .d(new_n201), .o1(new_n207));
  aoi112aa1n02x5               g112(.a(new_n206), .b(new_n205), .c(new_n176), .d(new_n201), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(\s[21] ));
  nor022aa1n04x5               g114(.a(\b[20] ), .b(\a[21] ), .o1(new_n210));
  xorc02aa1n12x5               g115(.a(\a[22] ), .b(\b[21] ), .out0(new_n211));
  nona22aa1n06x5               g116(.a(new_n207), .b(new_n211), .c(new_n210), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n211), .o1(new_n213));
  oaoi13aa1n06x5               g118(.a(new_n213), .b(new_n207), .c(\a[21] ), .d(\b[20] ), .o1(new_n214));
  norb02aa1n02x7               g119(.a(new_n212), .b(new_n214), .out0(\s[22] ));
  nano23aa1n09x5               g120(.a(new_n186), .b(new_n193), .c(new_n194), .d(new_n187), .out0(new_n216));
  nanp02aa1n02x5               g121(.a(new_n211), .b(new_n206), .o1(new_n217));
  nano22aa1n02x4               g122(.a(new_n217), .b(new_n183), .c(new_n216), .out0(new_n218));
  inv000aa1n02x5               g123(.a(new_n204), .o1(new_n219));
  inv000aa1d42x5               g124(.a(\a[21] ), .o1(new_n220));
  inv000aa1d42x5               g125(.a(\a[22] ), .o1(new_n221));
  xroi22aa1d04x5               g126(.a(new_n220), .b(\b[20] ), .c(new_n221), .d(\b[21] ), .out0(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n219), .c(new_n216), .d(new_n185), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\b[21] ), .o1(new_n224));
  oaoi03aa1n06x5               g129(.a(new_n221), .b(new_n224), .c(new_n210), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(new_n223), .b(new_n225), .o1(new_n226));
  tech160nm_fixorc02aa1n03p5x5 g131(.a(\a[23] ), .b(\b[22] ), .out0(new_n227));
  aoai13aa1n09x5               g132(.a(new_n227), .b(new_n226), .c(new_n176), .d(new_n218), .o1(new_n228));
  aoi112aa1n02x5               g133(.a(new_n227), .b(new_n226), .c(new_n176), .d(new_n218), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n228), .b(new_n229), .out0(\s[23] ));
  xorc02aa1n12x5               g135(.a(\a[24] ), .b(\b[23] ), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  oai112aa1n03x5               g137(.a(new_n228), .b(new_n232), .c(\b[22] ), .d(\a[23] ), .o1(new_n233));
  oaoi13aa1n02x5               g138(.a(new_n232), .b(new_n228), .c(\a[23] ), .d(\b[22] ), .o1(new_n234));
  norb02aa1n03x4               g139(.a(new_n233), .b(new_n234), .out0(\s[24] ));
  nanp02aa1n03x5               g140(.a(new_n231), .b(new_n227), .o1(new_n236));
  nano32aa1n02x4               g141(.a(new_n236), .b(new_n222), .c(new_n183), .d(new_n216), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n238));
  oab012aa1n02x4               g143(.a(new_n238), .b(\a[24] ), .c(\b[23] ), .out0(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n236), .c(new_n223), .d(new_n225), .o1(new_n240));
  aoi012aa1n03x5               g145(.a(new_n240), .b(new_n176), .c(new_n237), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[24] ), .b(\a[25] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  xnrc02aa1n03x5               g148(.a(new_n241), .b(new_n243), .out0(\s[25] ));
  aoai13aa1n06x5               g149(.a(new_n243), .b(new_n240), .c(new_n176), .d(new_n237), .o1(new_n245));
  xnrc02aa1n02x5               g150(.a(\b[25] ), .b(\a[26] ), .out0(new_n246));
  oai112aa1n03x5               g151(.a(new_n245), .b(new_n246), .c(\b[24] ), .d(\a[25] ), .o1(new_n247));
  oaoi13aa1n03x5               g152(.a(new_n246), .b(new_n245), .c(\a[25] ), .d(\b[24] ), .o1(new_n248));
  norb02aa1n02x7               g153(.a(new_n247), .b(new_n248), .out0(\s[26] ));
  nor042aa1n03x5               g154(.a(new_n246), .b(new_n242), .o1(new_n250));
  inv000aa1n03x5               g155(.a(new_n250), .o1(new_n251));
  nona32aa1n09x5               g156(.a(new_n201), .b(new_n251), .c(new_n236), .d(new_n217), .out0(new_n252));
  nanb02aa1n12x5               g157(.a(new_n252), .b(new_n176), .out0(new_n253));
  norp02aa1n02x5               g158(.a(\b[25] ), .b(\a[26] ), .o1(new_n254));
  aoi112aa1n02x5               g159(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n255));
  norp02aa1n02x5               g160(.a(new_n255), .b(new_n254), .o1(new_n256));
  aobi12aa1n09x5               g161(.a(new_n256), .b(new_n240), .c(new_n250), .out0(new_n257));
  norp02aa1n02x5               g162(.a(\b[26] ), .b(\a[27] ), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(\b[26] ), .b(\a[27] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  xnbna2aa1n06x5               g165(.a(new_n260), .b(new_n257), .c(new_n253), .out0(\s[27] ));
  inv000aa1n06x5               g166(.a(new_n258), .o1(new_n262));
  aobi12aa1n06x5               g167(.a(new_n260), .b(new_n257), .c(new_n253), .out0(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[27] ), .b(\a[28] ), .out0(new_n264));
  nano22aa1n03x5               g169(.a(new_n263), .b(new_n262), .c(new_n264), .out0(new_n265));
  oaoi13aa1n06x5               g170(.a(new_n252), .b(new_n175), .c(new_n122), .d(new_n171), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n225), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n236), .o1(new_n268));
  aoai13aa1n04x5               g173(.a(new_n268), .b(new_n267), .c(new_n205), .d(new_n222), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n256), .b(new_n251), .c(new_n269), .d(new_n239), .o1(new_n270));
  oaih12aa1n02x5               g175(.a(new_n260), .b(new_n270), .c(new_n266), .o1(new_n271));
  tech160nm_fiaoi012aa1n02p5x5 g176(.a(new_n264), .b(new_n271), .c(new_n262), .o1(new_n272));
  norp02aa1n03x5               g177(.a(new_n272), .b(new_n265), .o1(\s[28] ));
  nano22aa1n02x4               g178(.a(new_n264), .b(new_n262), .c(new_n259), .out0(new_n274));
  oaih12aa1n02x5               g179(.a(new_n274), .b(new_n270), .c(new_n266), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[28] ), .b(\b[27] ), .c(new_n262), .carry(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[28] ), .b(\a[29] ), .out0(new_n277));
  tech160nm_fiaoi012aa1n02p5x5 g182(.a(new_n277), .b(new_n275), .c(new_n276), .o1(new_n278));
  aobi12aa1n06x5               g183(.a(new_n274), .b(new_n257), .c(new_n253), .out0(new_n279));
  nano22aa1n03x5               g184(.a(new_n279), .b(new_n276), .c(new_n277), .out0(new_n280));
  norp02aa1n03x5               g185(.a(new_n278), .b(new_n280), .o1(\s[29] ));
  xorb03aa1n02x5               g186(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g187(.a(new_n277), .b(new_n264), .c(new_n259), .d(new_n262), .out0(new_n283));
  oaih12aa1n02x5               g188(.a(new_n283), .b(new_n270), .c(new_n266), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[29] ), .b(\b[28] ), .c(new_n276), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[29] ), .b(\a[30] ), .out0(new_n286));
  tech160nm_fiaoi012aa1n02p5x5 g191(.a(new_n286), .b(new_n284), .c(new_n285), .o1(new_n287));
  aobi12aa1n06x5               g192(.a(new_n283), .b(new_n257), .c(new_n253), .out0(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  norp02aa1n03x5               g194(.a(new_n287), .b(new_n289), .o1(\s[30] ));
  norb03aa1n02x5               g195(.a(new_n274), .b(new_n286), .c(new_n277), .out0(new_n291));
  oai012aa1n03x5               g196(.a(new_n291), .b(new_n270), .c(new_n266), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[30] ), .b(\b[29] ), .c(new_n285), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[30] ), .b(\a[31] ), .out0(new_n294));
  tech160nm_fiaoi012aa1n02p5x5 g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n06x5               g200(.a(new_n291), .b(new_n257), .c(new_n253), .out0(new_n296));
  nano22aa1n03x5               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  norp02aa1n03x5               g202(.a(new_n295), .b(new_n297), .o1(\s[31] ));
  xorb03aa1n02x5               g203(.a(new_n100), .b(\b[2] ), .c(new_n102), .out0(\s[3] ));
  xnrc02aa1n02x5               g204(.a(\b[3] ), .b(\a[4] ), .out0(new_n300));
  oao003aa1n02x5               g205(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .carry(new_n301));
  mtn022aa1n02x5               g206(.a(new_n301), .b(new_n106), .sa(new_n300), .o1(\s[4] ));
  and002aa1n02x5               g207(.a(\b[3] ), .b(\a[4] ), .o(new_n303));
  aboi22aa1n03x5               g208(.a(new_n303), .b(new_n106), .c(new_n113), .d(new_n112), .out0(new_n304));
  nona23aa1n02x4               g209(.a(new_n106), .b(new_n113), .c(new_n111), .d(new_n303), .out0(new_n305));
  norb02aa1n02x5               g210(.a(new_n305), .b(new_n304), .out0(\s[5] ));
  xobna2aa1n03x5               g211(.a(new_n110), .b(new_n305), .c(new_n112), .out0(\s[6] ));
  nona22aa1n02x4               g212(.a(new_n305), .b(new_n111), .c(new_n110), .out0(new_n308));
  nanb02aa1n02x5               g213(.a(new_n116), .b(new_n115), .out0(new_n309));
  xnbna2aa1n03x5               g214(.a(new_n309), .b(new_n308), .c(new_n108), .out0(\s[7] ));
  aoi013aa1n02x4               g215(.a(new_n116), .b(new_n308), .c(new_n108), .d(new_n115), .o1(new_n311));
  xnrb03aa1n02x5               g216(.a(new_n311), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g217(.a(new_n122), .b(new_n137), .out0(\s[9] ));
endmodule


